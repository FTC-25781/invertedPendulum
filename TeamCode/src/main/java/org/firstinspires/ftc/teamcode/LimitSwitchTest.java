package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Limit Switch Test", group = "")
public class LimitSwitchTest extends LinearOpMode {
    private DigitalChannel leftLimit;
    private DigitalChannel rightLimit;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftLimit = hardwareMap.get(DigitalChannel.class, "leftLimit");
        rightLimit = hardwareMap.get(DigitalChannel.class, "rightLimit");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left Limit Switch" , leftLimit.getState());
            telemetry.addData("Right Limit Switch" , rightLimit.getState());
            telemetry.update();
        }
    }
}
