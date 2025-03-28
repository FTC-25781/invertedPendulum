package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Inverted Pendulum", group = "")
public class InvertedPendulum extends LinearOpMode {

    private DcMotorEx motor;
    DigitalChannel leftLimit;
    DigitalChannel rightLimit;

    private double setPoint = 0;
    private double output = 0;

    private double kp, ki, kd;

    private double integral = 0;
    private double derivative = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
//        encoderMotor = hardwareMap.get(DcMotorEx.class, "encoderMotor");
        leftLimit = hardwareMap.get(DigitalChannel.class, "leftLimit");
        rightLimit = hardwareMap.get(DigitalChannel.class, "rightLimit");

        // Reset encoders
//        encoderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        encoderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Inversion logic and gains
            kp = 10.00;
            ki = 0.00;
            kd = 0.00;

        waitForStart();

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        timer.reset();

        while (opModeIsActive() && leftLimit.getState() && rightLimit.getState()) {
            int encoderCount = motor.getCurrentPosition();

            double error = setPoint - encoderCount;

            // Basic PID calculation
            integral += error;
            derivative = (error - lastError);

            output = kp * error + ki * integral + kd * derivative;
            lastError = error;

            // Clamp motor power (motors accept power from -1 to 1)
            double power = Math.max(-1.0, Math.min(1.0, output/100));

            motor.setPower(power);

            telemetry.addData("Output", output);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Error", error);
            telemetry.addData("Motor Value", motor.getCurrentPosition()); //anti-clockwise is negative
            telemetry.addData("Left Limit Switch" , leftLimit.getState());
            telemetry.addData("Right Limit Switch" , rightLimit.getState());
            telemetry.update();
        }

        motor.setPower(0);
    }
}
