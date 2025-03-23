package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class EncoderValue extends LinearOpMode {

    private DcMotorEx encoder;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        encoder = hardwareMap.get(DcMotorEx.class, "encoder");

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Encoder Pos", "power (%.2f)", encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
