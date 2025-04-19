package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Ball Balancer Using OpenCV")
public class BallBalancerUsingOpenCV extends LinearOpMode {

    private Servo servoX, servoY, servoZ;
    private OpenCvCamera webcam;
    private BallDetectionPipeline pipeline;

    // PID Constants
    double kp = 0.005, ki = 0.0001, kd = 0.001;
    double errorX_prev = 0, errorY_prev = 0;
    double integralX = 0, integralY = 0;

    // Platform settings
    double originX = 320, originY = 180;
    double platformRadius = 150.0;
    double maxTiltDistance = 5.0; // cm: max linear tilt radius

    // Arm link lengths (in cm)
    double L1 = 11.2;
    double L2 = 4.5;

    // Bounce phase for Z axis
    private double bouncePhase = 0;
    long previousTime = System.currentTimeMillis();

    @Override
    public void runOpMode() throws InterruptedException {
        servoX = hardwareMap.get(Servo.class, "servoX");
        servoY = hardwareMap.get(Servo.class, "servoY");
        servoZ = hardwareMap.get(Servo.class, "servoZ");

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        pipeline = new BallDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        originX = 640 / 2.0;
        originY = 360 / 2.0;

        while (opModeIsActive()) {
            double ballX = pipeline.ballX;
            double ballY = pipeline.ballY;

            if (Math.abs(ballX - originX) < 20 && Math.abs(ballY - originY) < 20) {
                originX = 0.9 * originX + 0.1 * ballX;
                originY = 0.9 * originY + 0.1 * ballY;
            }

            double errorX = originX - ballX;
            double errorY = originY - ballY;

            integralX = clamp(integralX + errorX, -100, 100);
            integralY = clamp(integralY + errorY, -100, 100);
            double derivativeX = errorX - errorX_prev;
            double derivativeY = errorY - errorY_prev;
            errorX_prev = errorX;
            errorY_prev = errorY;

            double outputX = kp * errorX + ki * integralX + kd * derivativeX;
            double outputY = kp * errorY + ki * integralY + kd * derivativeY;

            // Scale outputs to platform tilt range (in cm)
            double tiltX_cm = clamp(outputX, -maxTiltDistance, maxTiltDistance);
            double tiltY_cm = clamp(outputY, -maxTiltDistance, maxTiltDistance);

            // Inverse Kinematics for X and Y servos
            double servoXPos = inverseKinematicsToServo(tiltX_cm, L1, L2);
            double servoYPos = inverseKinematicsToServo(tiltY_cm, L1, L2);

            // Z axis bounce
            long currentTime = System.currentTimeMillis();
            long deltaTime = currentTime - previousTime;
            previousTime = currentTime;
            bouncePhase += 0.1 * deltaTime / 1000.0;
            if (bouncePhase > 2 * Math.PI) bouncePhase -= 2 * Math.PI;
            double zOffset = 0.05 * Math.sin(bouncePhase);
            double servoZPos = clamp(0.5 + zOffset, 0, 1);

            // Set servos
            servoX.setPosition(clamp(servoXPos, 0, 1));
            servoY.setPosition(clamp(servoYPos, 0, 1));
            servoZ.setPosition(servoZPos);

            telemetry.addData("Ball X", ballX);
            telemetry.addData("Ball Y", ballY);
            telemetry.addData("Tilt X (cm)", tiltX_cm);
            telemetry.addData("Tilt Y (cm)", tiltY_cm);
            telemetry.addData("Servo X", servoXPos);
            telemetry.addData("Servo Y", servoYPos);
            telemetry.addData("Servo Z", servoZPos);
            telemetry.update();

            sleep(50);
        }
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // Convert IK shoulder angle to servo position
    public double inverseKinematicsToServo(double x, double L1, double L2) {
        double y = 0; // Planar 2D
        double d = Math.sqrt(x * x + y * y);

        // Prevent domain error
        d = Math.min(d, L1 + L2 - 0.01);

        double angle2 = Math.acos((x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2));
        double angle1 = Math.atan2(y, x) - Math.atan2(L2 * Math.sin(angle2), L1 + L2 * Math.cos(angle2));

        double shoulderAngleDeg = Math.toDegrees(angle1);

        // Map angle to servo (shoulder angle range assumed to be [-45°, 45°])
        return (shoulderAngleDeg + 45) / 90.0;
    }

    public class BallDetectionPipeline extends OpenCvPipeline {
        public double ballX = 320;
        public double ballY = 180;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsvMat = new Mat();
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            Scalar lower = new Scalar(125, 50, 50);  // Tune for ball color
            Scalar upper = new Scalar(170, 255, 255);

            Mat mask = new Mat();
            Core.inRange(hsvMat, lower, upper, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            Rect boundingRect = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    boundingRect = Imgproc.boundingRect(contour);
                }
            }

            if (boundingRect != null) {
                ballX = boundingRect.x + boundingRect.width / 2.0;
                ballY = boundingRect.y + boundingRect.height / 2.0;
                Imgproc.circle(input, new Point(ballX, ballY), 5, new Scalar(0, 255, 0), -1);
            }

            return input;
        }
    }
}
