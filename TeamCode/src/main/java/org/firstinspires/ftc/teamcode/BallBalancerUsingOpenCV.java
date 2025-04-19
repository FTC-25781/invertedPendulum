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

@TeleOp(name = "Ball Balancer on Circular Platform")
public class BallBalancerUsingOpenCV extends LinearOpMode {
    private Servo servoX;
    private Servo servoY;
    private Servo servoZ;

    private OpenCvCamera webcam;
    private BallDetectionPipeline pipeline;

    // PID constants
    double kp = 0.005;
    double ki = 0.0001;
    double kd = 0.001;

    double errorX_prev = 0;
    double errorY_prev = 0;
    double integralX = 0;
    double integralY = 0;

    double originX;
    double originY;

    private double bouncePhase = 0.0;

    // Platform radius for inverse kinematics (assumes circular platform)
    private double platformRadius = 150.0;  // in same units as PID output scaling

    @Override
    public void runOpMode() throws InterruptedException {
        servoX = hardwareMap.get(Servo.class, "servoX");
        servoY = hardwareMap.get(Servo.class, "servoY");
        servoZ = hardwareMap.get(Servo.class, "servoZ");

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

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

        sleep(500);

        while (opModeIsActive()) {
            double ballX = pipeline.ballX;
            double ballY = pipeline.ballY;

            // Auto-calibration: slowly adjust origin if ball is near center by calculating if within 20 pixels (compares pixel coordinates)
            if (Math.abs(ballX - originX) < 20 && Math.abs(ballY - originY) < 20) {
                originX = 0.9 * originX + 0.1 * ballX; //keeps 90% of the origin and shifts 10% to the ball for new origin
                originY = 0.9 * originY + 0.1 * ballY;
            }

            double errorX = originX - ballX;
            double errorY = originY - ballY;

            // PID for X-axis
            integralX += errorX;
            integralX = Math.max(-100, Math.min(100, integralX));
            double derivativeX = errorX - errorX_prev;
            double outputX = kp * errorX + ki * integralX + kd * derivativeX;
            errorX_prev = errorX;

            // PID for Y-axis
            integralY += errorY;
            integralY = Math.max(-100, Math.min(100, integralY));
            double derivativeY = errorY - errorY_prev;
            double outputY = kp * errorY + ki * integralY + kd * derivativeY;
            errorY_prev = errorY;

            // Inverse kinematics:
            // compute tilt angles by treating PID outputs as desired displacement
            // and platformRadius as the horizontal distance for the angle calculation
            double angleX = Math.atan2(outputX, platformRadius);
            double angleY = Math.atan2(outputY, platformRadius);

            double servoXPosition = Math.max(0, Math.min(1, 0.5 + angleX));
            double servoYPosition = Math.max(0, Math.min(1, 0.5 + angleY));

            // Z-axis motion
            bouncePhase += 0.1; // increases phase of sine wave and 0.1 controls speed of the bounce
            if (bouncePhase > 2 * Math.PI) bouncePhase = 0; // resets phase of sine wave
            double zOffset = 0.05 * Math.sin(bouncePhase); // computes vertical bounce amount by keeping it between 0.5 and -0.5
            double servoZPosition = Math.max(0, Math.min(1, 0.5 + zOffset)); // uses zOffset to set the servo position

            servoX.setPosition(servoXPosition);
            servoY.setPosition(servoYPosition);
            servoZ.setPosition(servoZPosition);

            telemetry.addData("Ball X", ballX);
            telemetry.addData("Ball Y", ballY);
            telemetry.addData("Error X", errorX);
            telemetry.addData("Error Y", errorY);
            telemetry.addData("Servo X Pos", servoXPosition);
            telemetry.addData("Servo Y Pos", servoYPosition);
            telemetry.addData("Servo Z Pos (Bounce)", servoZPosition);
            telemetry.update();

            sleep(50);
        }
    }

    public class BallDetectionPipeline extends OpenCvPipeline {
        public double ballX = 160;
        public double ballY = 120;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsvMat = new Mat();
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            Scalar lowerBound = new Scalar(100, 100, 50); // Adjust to match ball color
            Scalar upperBound = new Scalar(140, 255, 255);
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
            } else {
                ballX = originX;
                ballY = originY;
            }

            return input;
        }
    }
}


