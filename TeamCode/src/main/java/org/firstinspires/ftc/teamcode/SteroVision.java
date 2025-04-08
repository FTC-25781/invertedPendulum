package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.calib3d.StereoBM;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Stereo Vision Demo")
public class SteroVision extends LinearOpMode {

    @Override
    public void runOpMode() {

        int leftCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int rightCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera leftCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "leftCam"), leftCameraMonitorViewId);
        OpenCvCamera rightCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "rightCam"), rightCameraMonitorViewId);

        leftCamera.openCameraDevice();
        rightCamera.openCameraDevice();

        StereoProcessor processor = new StereoProcessor();

        leftCamera.setPipeline(processor.leftPipeline);
        rightCamera.setPipeline(processor.rightPipeline);

        leftCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        rightCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            // Display a message on the driver station
            telemetry.addLine("Streaming from both cameras...");
            telemetry.update();
            sleep(100);  // Sleep for 100ms to reduce CPU load
        }

        leftCamera.stopStreaming();
        rightCamera.stopStreaming();
    }

    static class StereoProcessor {
        Mat leftImage = new Mat();
        Mat rightImage = new Mat();
        Mat disparity = new Mat();

        OpenCvPipeline leftPipeline = new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                input.copyTo(leftImage);
                return input;
            }
        };

        OpenCvPipeline rightPipeline = new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                input.copyTo(rightImage);
                if (!leftImage.empty() && !rightImage.empty()) {
                    // Convert to grayscale
                    Imgproc.cvtColor(leftImage, leftImage, Imgproc.COLOR_RGB2GRAY);
                    Imgproc.cvtColor(rightImage, rightImage, Imgproc.COLOR_RGB2GRAY);

                    // Compute disparity map
                    StereoBM stereoBM = StereoBM.create(16, 15);
                    stereoBM.compute(leftImage, rightImage, disparity);

                    // Apply a color map to visualize the disparity map
                    Imgproc.applyColorMap(disparity, disparity, Imgproc.COLORMAP_JET);
                }
                return input;
            }
        };
    }
}