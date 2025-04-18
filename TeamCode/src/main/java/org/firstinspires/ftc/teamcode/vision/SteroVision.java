package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.StereoBM;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SteroVision{
    Mat leftImage = new Mat();
    Mat rightImage = new Mat();
    Mat disparity = new Mat();
    private volatile int frameCount = 0;
    private final String label;

    public SteroVision(String label) { this.label = label; }

    VisionProcessor leftPipeline = new VisionProcessor() {
        @Override
        public void init(int width, int height, CameraCalibration calibration) { }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            frame.copyTo(leftImage);
            Imgproc.cvtColor(leftImage, leftImage, Imgproc.COLOR_RGB2GRAY);
            return frame;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) { }
    };

    VisionProcessor rightPipeline = new VisionProcessor() {
        @Override
        public void init(int width, int height, CameraCalibration calibration) {  }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            frame.copyTo(rightImage);
            if (!leftImage.empty() && !rightImage.empty()) {
                // Convert to grayscale
                Imgproc.cvtColor(rightImage, rightImage, Imgproc.COLOR_RGB2GRAY);

                // Compute disparity map
                StereoBM stereoBM = StereoBM.create(16, 15);
                stereoBM.compute(rightImage, frame, frame);

                // Apply a color map to visualize the disparity map
                Imgproc.applyColorMap(disparity, frame, Imgproc.COLORMAP_JET);
            }
            return frame;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) { }
    };
}
