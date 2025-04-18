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
    private Servo servoX;
    private Servo servoY;

    private OpenCvCamera webcam;
    private BallDetectionPipeline pipeline;

    // PID constants for X, Y
    double kp = 0.005;
    double ki = 0.0;
    double kd = 0.001;

    double errorX_prev = 0;
    double errorY_prev = 0;

    double integralX = 0;
    double integralY = 0;

    // Origin is the center of the platform (camera frame)
    double originX;
    double originY;

    @Override
    public void runOpMode() throws InterruptedException {
        servoX = hardwareMap.get(Servo.class, "servoX");
        servoY = hardwareMap.get(Servo.class, "servoY");

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

        // Set the origin at the center of the camera's frame
        originX = 320 / 2.0; // Horizontal center of the camera frame
        originY = 240 / 2.0; // Vertical center of the camera frame

        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double ballX = pipeline.ballX;
            double ballY = pipeline.ballY;

            // Calculate the error in X and Y directions
            double errorX = originX - ballX;
            double errorY = originY - ballY;

            // PID control for X-axis
            integralX += errorX;
            double derivativeX = errorX - errorX_prev;
            double outputX = kp * errorX + ki * integralX + kd * derivativeX;
            errorX_prev = errorX;

            // PID control for Y-axis
            integralY += errorY;
            double derivativeY = errorY - errorY_prev;
            double outputY = kp * errorY + ki * integralY + kd * derivativeY;
            errorY_prev = errorY;

            // Inverse Kinematics:
            double servoXPosition = 0.5 + outputX;
            double servoYPosition = 0.5 + outputY;

            servoXPosition = Math.max(0, Math.min(1, servoXPosition));
            servoYPosition = Math.max(0, Math.min(1, servoYPosition));

            servoX.setPosition(servoXPosition);
            servoY.setPosition(servoYPosition);

            telemetry.addData("Ball X", ballX);
            telemetry.addData("Ball Y", ballY);
            telemetry.addData("Error X", errorX);
            telemetry.addData("Error Y", errorY);
            telemetry.addData("Servo X Pos", servoXPosition);
            telemetry.addData("Servo Y Pos", servoYPosition);
            telemetry.update();

            sleep(50); // for stability
        }
    }

    public class BallDetectionPipeline extends OpenCvPipeline {
        public double ballX = 160;
        public double ballY = 120;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsvMat = new Mat();
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV); // converts RGB to HSV

            Scalar lowerBound = new Scalar(0, 100, 100);
            Scalar upperBound = new Scalar(10, 255, 255);
            Mat mask = new Mat();
            Core.inRange(hsvMat, lowerBound, upperBound, mask); // creates binary mask

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE); // Find all edges of a blob

            double maxArea = 0;
            Rect boundingRect = null;
            for (MatOfPoint contour : contours) { // finding largest contour (assume to be ball)
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
