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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera leftCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "LeftCamera"), cameraMonitorViewId);
        OpenCvCamera rightCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RightCamera"), cameraMonitorViewId);

        leftCamera.openCameraDevice();
        rightCamera.openCameraDevice();

        StereoProcessor processor = new StereoProcessor();
        leftCamera.setPipeline(processor.leftPipeline);
        rightCamera.setPipeline(processor.rightPipeline);

        leftCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        rightCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Streaming from both cameras...");
            telemetry.update();
            sleep(100);
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
                    Imgproc.cvtColor(leftImage, leftImage, Imgproc.COLOR_RGB2GRAY);
                    Imgproc.cvtColor(rightImage, rightImage, Imgproc.COLOR_RGB2GRAY);

                    StereoBM stereoBM = StereoBM.create(16, 15);  // Parameters can be tuned
                    stereoBM.compute(leftImage, rightImage, disparity);
                }
                return input;
            }
        };
    }
}
