package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Blue Object Lock with Servo X", group = "Linear Opmode")
public class CameraOpMode extends LinearOpMode {

    private OpenCvCamera cvCamera;
    private Servo spinServo;
    private Servo vertServo;
    private Mat hsvMat = new Mat();
    private Mat thresholdMat = new Mat();

    // Blue color range in HSV
    private static final Scalar BLUE_LOWER = new Scalar(100, 150, 50);
    private static final Scalar BLUE_UPPER = new Scalar(140, 255, 255);

    // Servo range and initial position
    private double spinServoPosition = 0.5;
    private double vertServoPosition = 0.5;

    // Frame dimensions and center
    private final int FRAME_WIDTH = 640;
    private final int FRAME_HEIGHT = 480;
    private final int CENTER_X = FRAME_WIDTH / 2;
    private final int CENTER_Y = FRAME_HEIGHT / 2;

    // Dead zone for error margin
    private final int DEAD_ZONE = 50;

    // Servo correction step (slow, smooth adjustments)
    private final double CORRECTION_STEP = 0.005;

    // Locking variables
    private boolean lockedOn = false;
    private Rect lockedBlock = null;

    @Override
    public void runOpMode() {
        // Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cvCamera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize servo
        spinServo = hardwareMap.get(Servo.class, "horizontalServo");
        spinServo.setPosition(spinServoPosition);  // Set initial position

        vertServo = hardwareMap.get(Servo.class, "verticalServo");
        vertServo.setPosition(vertServoPosition);

        // Set OpenCV pipeline
        cvCamera.setPipeline(new BlueBlockDetectionPipeline());

        // Open camera asynchronously
        cvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cvCamera.startStreaming(FRAME_WIDTH, FRAME_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            if (lockedOn && lockedBlock != null) {
                double objectCenterX = lockedBlock.x + (lockedBlock.width / 2.0);
                double errorX = objectCenterX - CENTER_X;
                double dynamicStep = CORRECTION_STEP * (Math.abs(errorX) / (FRAME_WIDTH / 2.0));
                double dampingFactor = Math.max(0.2, 1.0 - (Math.abs(errorX) / (FRAME_WIDTH / 4.0)));
                dynamicStep *= dampingFactor;

                if (errorX > DEAD_ZONE) spinServoPosition += dynamicStep;
                else if (errorX < -DEAD_ZONE) spinServoPosition -= dynamicStep;

                double objectCenterY = lockedBlock.y + (lockedBlock.height / 2.0);
                double errorY = objectCenterY - CENTER_Y;
                double dynamicStepY = CORRECTION_STEP * (Math.abs(errorY) / (FRAME_HEIGHT / 2.0));
                double dampingFactorY = Math.max(0.2, 1.0 - (Math.abs(errorY) / (FRAME_HEIGHT / 4.0)));
                dynamicStepY *= dampingFactorY;

                if (errorY > DEAD_ZONE) vertServoPosition += dynamicStepY;
                else if (errorY < -DEAD_ZONE) vertServoPosition -= dynamicStepY;

                spinServo.setPosition(spinServoPosition);
                vertServo.setPosition(vertServoPosition);

                telemetry.addData("Locked On", "Yes");
                telemetry.addData("Object Center X", objectCenterX - CENTER_X);
                telemetry.addData("Object Center Y", objectCenterY - CENTER_Y);
                telemetry.addData("Servo Position X", spinServoPosition);
                telemetry.addData("Servo Position Y", vertServoPosition);
            } else {
                telemetry.addData("Locked On", "No");
            }

            telemetry.update();
        }
    }

    // OpenCV pipeline to detect blue objects
    private class BlueBlockDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Create binary mask for blue detection
            Core.inRange(hsvMat, BLUE_LOWER, BLUE_UPPER, thresholdMat);

            // Find contours in the thresholded image
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (contours.size() > 0) {
                double maxArea = 0;
                int maxAreaIndex = -1;

                // Find largest contour
                for (int i = 0; i < contours.size(); i++) {
                    double area = Imgproc.contourArea(contours.get(i));
                    if (area > maxArea) {
                        maxArea = area;
                        maxAreaIndex = i;
                    }
                }

                if (maxAreaIndex >= 0) {
                    lockedBlock = Imgproc.boundingRect(contours.get(maxAreaIndex));
                    lockedOn = true;

                    // Draw bounding box, center point, and axis
                    drawAxisAndCenter(input);
                    drawBoundingBoxAndCenter(input, lockedBlock);
                } else {
                    resetLock();
                }
            } else {
                resetLock();
            }

            hierarchy.release();
            return input;
        }

        // Draw bounding box and center point
        private void drawBoundingBoxAndCenter(Mat input, Rect block) {
            // Draw bounding box
            Imgproc.rectangle(input, block.tl(), block.br(), new Scalar(0, 255, 0), 2);

            // Draw center point of the object
            int objectCenterX = block.x + (block.width / 2);
            int objectCenterY = block.y + (block.height / 2);
            Imgproc.circle(input, new Point(objectCenterX, objectCenterY), 5, new Scalar(0, 0, 255), -1);

            // Draw line from (0,0) to object center
            Imgproc.line(input, new Point(CENTER_X, CENTER_Y), new Point(objectCenterX, objectCenterY), new Scalar(0, 255, 0), 2);
        }

        // Draw axis (coordinate plane) with (0,0) at the center
        private void drawAxisAndCenter(Mat input) {
            // Draw X-axis
            Imgproc.line(input, new Point(0, CENTER_Y), new Point(FRAME_WIDTH, CENTER_Y), new Scalar(255, 0, 0), 1);
            // Draw Y-axis
            Imgproc.line(input, new Point(CENTER_X, 0), new Point(CENTER_X, FRAME_HEIGHT), new Scalar(255, 0, 0), 1);
        }

        // Reset lock when object is lost
        private void resetLock() {
            lockedOn = false;
            lockedBlock = null;
        }
    }

    // Clamp servo position between min and max values
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}