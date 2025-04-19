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

@TeleOp(name = "Ball Balancer with IK (Angle)")
public class BallBalancerUsingOpenCV extends LinearOpMode {

    private Servo servoX, servoY, servoZ;
    private OpenCvCamera webcam;
    private BallDetectionPipeline pipeline;

    // PID Constants
    double kp = 0.005, ki = 0.0001, kd = 0.001;
    double errorX_prev = 0, errorY_prev = 0;
    double integralX = 0, integralY = 0;

    // Platform settings
    double originX = 320, originY = 180; // Center of 640x360 frame
    double platformRadius = 150.0;
    double maxAngleDegrees = 30.0; // Max tilt in degrees

    // Bounce phase
    private double bouncePhase = 0;
    long previousTime = System.currentTimeMillis();  // To track the previous time

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
                telemetry.addData("Error", "Camera open error: " + errorCode);
            }
        });

        originX = 640 / 2.0;
        originY = 360 / 2.0;

        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double ballX = pipeline.ballX;
            double ballY = pipeline.ballY;

            if (Math.abs(ballX - originX) < 20 && Math.abs(ballY - originY) < 20) {
                originX = 0.9 * originX + 0.1 * ballX;
                originY = 0.9 * originY + 0.1 * ballY;
            }

            double errorX = originX - ballX;
            double errorY = originY - ballY;

            // PID Control
            integralX = clamp(integralX + errorX, -100, 100);
            integralY = clamp(integralY + errorY, -100, 100);
            double derivativeX = errorX - errorX_prev;
            double derivativeY = errorY - errorY_prev;
            errorX_prev = errorX;
            errorY_prev = errorY;

            double outputX = kp * errorX + ki * integralX + kd * derivativeX;
            double outputY = kp * errorY + ki * integralY + kd * derivativeY;

            // Inverse Kinematics: compute tilt angles in radians
            double angleX_rad = Math.atan2(outputX, platformRadius);
            double angleY_rad = Math.atan2(outputY, platformRadius);

            // Convert radians to degrees
            double angleX_deg = Math.toDegrees(angleX_rad);
            double angleY_deg = Math.toDegrees(angleY_rad);

            // Clamp angles to safe tilt range
            angleX_deg = clamp(angleX_deg, -maxAngleDegrees, maxAngleDegrees);
            angleY_deg = clamp(angleY_deg, -maxAngleDegrees, maxAngleDegrees);

            // Map angles to servo position range (0.0 to 1.0)
            double servoXPosition = mapAngleToServo(angleX_deg, maxAngleDegrees);
            double servoYPosition = mapAngleToServo(angleY_deg, maxAngleDegrees);

            // Z Bounce (sine wave motion)
            // Calculate delta time (time difference between current and previous loop)
            long currentTime = System.currentTimeMillis();
            long deltaTime = currentTime - previousTime;
            previousTime = currentTime;  // Update the previous time

            // Z Bounce Control (using deltaTime)
            bouncePhase += 0.1 * deltaTime / 1000.0;  // Scale by deltaTime
            if (bouncePhase > 2 * Math.PI) {
                bouncePhase -= 2 * Math.PI;  // Ensure bouncePhase stays within a full sine wave cycle
            }
            // Calculate Z-axis servo position based on sine wave
            double zOffset = 0.05 * Math.sin(bouncePhase);
            double servoZPosition = clamp(0.5 + zOffset, 0, 1);

            servoX.setPosition(servoXPosition);
            servoY.setPosition(servoYPosition);
            servoZ.setPosition(servoZPosition);

            telemetry.addData("Ball X", ballX);
            telemetry.addData("Ball Y", ballY);
            telemetry.addData("Angle X (°)", angleX_deg);
            telemetry.addData("Angle Y (°)", angleY_deg);
            telemetry.addData("Servo X", servoXPosition);
            telemetry.addData("Servo Y", servoYPosition);
            telemetry.addData("Servo Z (Bounce)", servoZPosition);
            telemetry.update();

            sleep(50);
        }
    }
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public static double mapAngleToServo(double angleDeg, double maxAngleDeg) {
        // Map angle [-max, max] to [0, 1] servo position
        return (angleDeg + maxAngleDeg) / (2 * maxAngleDeg);
    }

    public class BallDetectionPipeline extends OpenCvPipeline {
        public double ballX = 320;
        public double ballY = 180;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsvMat = new Mat();
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            Scalar lowerBound = new Scalar(125, 50, 50); // Adjust to match ball color
            Scalar upperBound = new Scalar(170, 255, 255);

            Mat mask = new Mat();
            Core.inRange(hsvMat, lowerBound, upperBound, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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
