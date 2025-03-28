package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends LinearOpMode {
    private DcMotor Mot1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Mot1 = hardwareMap.get(DcMotor.class, "motor");
        Mot1.setDirection(DcMotor.Direction.FORWARD);
        Mot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            double power = gamepad1.left_stick_y;
            Mot1.setPower(power);
            telemetry.addData("Motor 1", "power (%.2f)", Mot1.getPower());
            telemetry.addData("Encoder Pos: ", Mot1.getCurrentPosition());
            telemetry.update();
        }
    }
}
