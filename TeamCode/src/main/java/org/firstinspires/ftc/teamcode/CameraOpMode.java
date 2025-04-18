package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

@TeleOp(name = "Color Sensor + Camera Lock", group = "Linear Opmode")
public class CameraOpMode extends LinearOpMode {

    private OpenCvCamera cvCamera;
    private Servo spinServo;
    private Servo vertServo;
    private ColorSensor colorSensor;

    private Mat hsvMat = new Mat();
    private Mat thresholdMat = new Mat();

    private float[] liveHSV = new float[3];
    private Scalar lowerHSV = new Scalar(100, 150, 50);
    private Scalar upperHSV = new Scalar(140, 255, 255);

    private double spinServoPosition = 0.5;
    private double vertServoPosition = 0.5;

    private final int FRAME_WIDTH = 640;
    private final int FRAME_HEIGHT = 480;
    private final int CENTER_X = FRAME_WIDTH / 2;
    private final int CENTER_Y = FRAME_HEIGHT / 2;
    private final int DEAD_ZONE = 50;
    private final double CORRECTION_STEP = 0.005;

    private boolean lockedOn = false;
    private Rect lockedBlock = null;

    private BlueBlockDetectionPipeline pipeline = new BlueBlockDetectionPipeline();

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        cvCamera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        spinServo = hardwareMap.get(Servo.class, "horizontalServo");
        spinServo.setPosition(spinServoPosition);

        vertServo = hardwareMap.get(Servo.class, "verticalServo");
        vertServo.setPosition(vertServoPosition);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        cvCamera.setPipeline(pipeline);

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
            // Get live RGB values and convert to HSV
            int r = colorSensor.red() * 8;
            int g = colorSensor.green() * 8;
            int b = colorSensor.blue() * 8;
            Color.RGBToHSV(r, g, b, liveHSV);

            float hue = liveHSV[0];       // 0-360
            float sat = liveHSV[1];       // 0-1
            float val = liveHSV[2];       // 0-1

            lowerHSV = new Scalar(hue - 10, Math.max(50, sat * 255 - 50), Math.max(50, val * 255 - 50));
            upperHSV = new Scalar(hue + 10, Math.min(255, sat * 255 + 50), Math.min(255, val * 255 + 50));

            // Send HSV to pipeline
            pipeline.setHSVRange(lowerHSV, upperHSV);

            if (lockedOn && lockedBlock != null) {
                double objectCenterX = lockedBlock.x + (lockedBlock.width / 2.0);
                double errorX = objectCenterX - CENTER_X;
                double dynamicStep = CORRECTION_STEP * (Math.abs(errorX) / (FRAME_WIDTH / 2.0));
                double dampingFactor = Math.max(0.2, 1.0 - (Math.abs(errorX) / (FRAME_WIDTH / 4.0)));
                dynamicStep *= dampingFactor;

                if (Math.abs(errorX) > DEAD_ZONE) {
                    spinServoPosition += (errorX > 0) ? dynamicStep : -dynamicStep;
                }

                double objectCenterY = lockedBlock.y + (lockedBlock.height / 2.0);
                double errorY = objectCenterY - CENTER_Y;
                double dynamicStepY = CORRECTION_STEP * (Math.abs(errorY) / (FRAME_HEIGHT / 2.0));
                double dampingFactorY = Math.max(0.2, 1.0 - (Math.abs(errorY) / (FRAME_HEIGHT / 4.0)));
                dynamicStepY *= dampingFactorY;

                if (Math.abs(errorY) > DEAD_ZONE) {
                    vertServoPosition += (errorY > 0) ? dynamicStepY : -dynamicStepY;
                }

                spinServo.setPosition(spinServoPosition / 5 + 0.5);
                vertServo.setPosition(vertServoPosition / 5 + 0.5);

                telemetry.addData("Object Offset X", objectCenterX - CENTER_X);
                telemetry.addData("Object Offset Y", objectCenterY - CENTER_Y);
                telemetry.addData("HSV", "H: %.1f S: %.1f V: %.1f", hue, sat, val);
                telemetry.addData("Locked On", "Yes");
            } else {
                telemetry.addData("Locked On", "No");
            }

            telemetry.update();
        }
    }

    private class BlueBlockDetectionPipeline extends OpenCvPipeline {
        private Scalar lowerHSV = new Scalar(100, 150, 50);
        private Scalar upperHSV = new Scalar(140, 255, 255);

        public void setHSVRange(Scalar lower, Scalar upper) {
            this.lowerHSV = lower;
            this.upperHSV = upper;
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, lowerHSV, upperHSV, thresholdMat);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
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
                    lockedBlock = Imgproc.boundingRect(contours.get(maxAreaIndex));
                    lockedOn = true;
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

        private void drawBoundingBoxAndCenter(Mat input, Rect block) {
            Imgproc.rectangle(input, block.tl(), block.br(), new Scalar(0, 255, 0), 2);
            int centerX = block.x + (block.width / 2);
            int centerY = block.y + (block.height / 2);
            Imgproc.circle(input, new Point(centerX, centerY), 5, new Scalar(0, 0, 255), -1);
            Imgproc.line(input, new Point(CENTER_X, CENTER_Y), new Point(centerX, centerY), new Scalar(0, 255, 0), 2);
        }

        private void drawAxisAndCenter(Mat input) {
            Imgproc.line(input, new Point(0, CENTER_Y), new Point(FRAME_WIDTH, CENTER_Y), new Scalar(255, 0, 0), 1);
            Imgproc.line(input, new Point(CENTER_X, 0), new Point(CENTER_X, FRAME_HEIGHT), new Scalar(255, 0, 0), 1);
        }

        private void resetLock() {
            lockedOn = false;
            lockedBlock = null;
        }
    }
}
