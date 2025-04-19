package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

@TeleOp(name = "Dual Camera VisionProcessor", group = "Vision")
public class SteroVisionOpMode extends LinearOpMode {
    VisionPortal portalLeft, portalRight;
    VisionProcessor processorLeft, processorRight;

    @Override
    public void runOpMode() {
        SteroVision processor = new SteroVision("StereoVision");
        telemetry.setMsTransmissionInterval(50);

        int[] viewIDs = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        portalLeft = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "leftCam"))
                .addProcessor(processor.leftPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(viewIDs[0])
                .build();

        portalLeft.setProcessorEnabled(processor.leftPipeline, true);


        portalRight = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "rightCam"))
                .addProcessor(processor.rightPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(viewIDs[1])
                .build();

        portalRight.setProcessorEnabled(processor.rightPipeline, true);

        telemetry.addLine("Both cameras initialized.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Camera 1 Frame", "");
            telemetry.addData("Camera 2 Frame", "");
            telemetry.update();
            sleep(100);
        }

        portalLeft.close();
        portalRight.close();
    }
}
