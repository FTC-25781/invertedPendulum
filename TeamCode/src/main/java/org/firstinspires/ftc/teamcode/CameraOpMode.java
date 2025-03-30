package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Object Tracking", group = "Linear Opmode")
public class CameraOpMode extends LinearOpMode {

    private Servo spinServo;
    private Servo upDownServo;
    private OpenCvCamera cvCamera;

    private Mat hsvMat = new Mat();
    private Mat thresholdMat = new Mat();

    // Blue color range in HSV
    private static final Scalar BLUE_LOWER = new Scalar(100, 150, 50);
    private static final Scalar BLUE_UPPER = new Scalar(140, 255, 255);

    // Servo range limits
    private final double SPIN_MIN = 0.0;
    private final double SPIN_MAX = 1.0;
    private final double UP_DOWN_MIN = 0.3;
    private final double UP_DOWN_MAX = 0.7;

    // Servo positions
    private double spinServoPosition = 0.5;
    private double upDownServoPosition = 0.5;

    // Correction step for adjustments
    private final double CORRECTION_STEP = 0.01;  // Increase for smoother correction

    // Locking mechanism variables
    private boolean lockedOn = false;
    private Rect lockedBlock = null;
    private int lostFrames = 0;
    private final int LOST_FRAME_THRESHOLD = 20; // Reset lock after losing object for 20 frames

    // ROI boundaries
    private final int ROI_MARGIN = 40;  // Region of Interest around the locked object

    private double frameCenterX = 320; // Center of 640x480 frame
    private double frameCenterY = 240;

    @Override
    public void runOpMode() {
        // Initialize servos
        spinServo = hardwareMap.get(Servo.class, "spin_servo");
        upDownServo = hardwareMap.get(Servo.class, "up_down_servo");

        // Set initial servo positions
        spinServo.setPosition(spinServoPosition);
        upDownServo.setPosition(upDownServoPosition);

        // Setup the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        cvCamera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        cvCamera.setPipeline(new BlueBlockDetectionPipeline());

        // Open camera asynchronously
        cvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cvCamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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
                // Track the locked object only if detected
                double objectCenterX = lockedBlock.x + (lockedBlock.width / 2.0);
                double objectCenterY = lockedBlock.y + (lockedBlock.height / 2.0);

                // Move servos to keep object in center
                adjustServo(objectCenterX, objectCenterY);

                lostFrames = 0;  // Reset lost frame counter
            } else {
                lostFrames++;
                if (lostFrames >= LOST_FRAME_THRESHOLD) {
                    resetLock();  // Reset if the object is lost for too long
                }
            }

            telemetry.addData("Locked On", lockedOn);
            telemetry.addData("Spin Servo", spinServoPosition);
            telemetry.addData("Up/Down Servo", upDownServoPosition);
            telemetry.update();
        }

        hsvMat.release();
        thresholdMat.release();
    }

    // Move servos based on the object's position to center it
    private void adjustServo(double objectCenterX, double objectCenterY) {
        // Only adjust if the object is outside the specified dead zone (30 pixels)
        final int DEAD_ZONE = 30;

        if (Math.abs(objectCenterX - frameCenterX) > DEAD_ZONE) {
            if (objectCenterX > frameCenterX) {
                spinServoPosition += CORRECTION_STEP;  // Move right
            } else {
                spinServoPosition -= CORRECTION_STEP;  // Move left
            }
        }

        if (Math.abs(objectCenterY - frameCenterY) > DEAD_ZONE) {
            if (objectCenterY > frameCenterY) {
                upDownServoPosition += CORRECTION_STEP;  // Move down
            } else {
                upDownServoPosition -= CORRECTION_STEP;  // Move up
            }
        }

        // Clamp the servo positions to prevent overshooting
        spinServoPosition = clamp(spinServoPosition, SPIN_MIN, SPIN_MAX);
        upDownServoPosition = clamp(upDownServoPosition, UP_DOWN_MIN, UP_DOWN_MAX);

        // Set the servo positions
        spinServo.setPosition(spinServoPosition);
        upDownServo.setPosition(upDownServoPosition);
    }

    // Reset lock and search for a new object
    private void resetLock() {
        lockedOn = false;
        lockedBlock = null;
        lostFrames = 0;
    }

    // Clamp a value between a min and max range
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // OpenCV pipeline for detecting blue blocks
    private class BlueBlockDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Create a binary mask where blue is detected
            Core.inRange(hsvMat, BLUE_LOWER, BLUE_UPPER, thresholdMat);

            // Find contours in the thresholded image
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (contours.size() > 0) {
                double maxArea = 0;
                int maxAreaIndex = -1;

                for (int i = 0; i < contours.size(); i++) {
                    double area = Imgproc.contourArea(contours.get(i));
                    if (area > maxArea) {
                        maxArea = area;
                        maxAreaIndex = i;
                    }
                }

                if (maxAreaIndex >= 0) {
                    Rect boundingRect = Imgproc.boundingRect(contours.get(maxAreaIndex));

                    if (!lockedOn) {
                        // Lock onto the first object found
                        lockedBlock = boundingRect;
                        lockedOn = true;
                    } else if (lockedOn && isSameBlock(boundingRect)) {
                        // Update the bounding box if the same object is still detected
                        lockedBlock = boundingRect;
                    }

                    // Draw bounding boxes
                    if (lockedOn && boundingRect.equals(lockedBlock)) {
                        Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(255, 0, 0), 2); // Red box for locked object
                    } else {
                        Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2); // Green box for others
                    }
                }
            } else {
                resetLock();  // Reset if no contour is found
            }

            hierarchy.release();
            return input;
        }

        // Check if the new bounding box is close enough to the locked object
        private boolean isSameBlock(Rect newBlock) {
            if (lockedBlock == null) {
                return false;
            }
            double distance = Math.sqrt(Math.pow(newBlock.x - lockedBlock.x, 2) + Math.pow(newBlock.y - lockedBlock.y, 2));
            return distance < 50; // Threshold to consider the same object
        }
    }
}
