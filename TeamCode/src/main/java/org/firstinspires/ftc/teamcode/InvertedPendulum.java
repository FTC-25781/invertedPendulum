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

    private double setPoint = 0.0;
    private double targetPosition = 0.0;
    private double Kp = 50.0;
    private double Ki = 0.0;
    private double Kd = 1.0;

    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        leftLimit = hardwareMap.get(DigitalChannel.class, "leftLimit");
        rightLimit = hardwareMap.get(DigitalChannel.class, "rightLimit");


        waitForStart();

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lastTime = System.currentTimeMillis();

        while (opModeIsActive() && leftLimit.getState() && rightLimit.getState()) {
            int encoderCount = motor.getCurrentPosition();

            double error = setPoint - encoderCount;

            // Basic PID calculation
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastTime) / 1000.0;

            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            double output = Kp * error + Ki * integral + Kd * derivative;

            lastError = error;
            lastTime = currentTime;

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
