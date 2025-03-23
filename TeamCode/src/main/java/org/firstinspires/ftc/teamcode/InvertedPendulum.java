package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Inverted Pendulum FTC", group = "Test")
public class InvertedPendulum extends LinearOpMode {

    private DcMotorEx motor;
    private DcMotorEx encoderMotor;

    private double setPoint = 0;
    private double input = 0;
    private double output = 0;

    private double kp, ki, kd;
    private double scaleFactor;
    private int outputDir;

    private double stepsPerMM;

    private double integral = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    // Constants
    private final int stepMode = 3;
    private final int[][] stepModes = {
            {5, 1, 0, 0, 0},
            {10, 2, 1, 0, 0},
            {20, 4, 0, 1, 0},
            {40, 8, 1, 1, 0},
            {80, 16, 0, 0, 1},
            {160, 32, 1, 1, 1}
    };

    private final boolean inverted = false;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        encoderMotor = hardwareMap.get(DcMotorEx.class, "encoderMotor");

        // Reset encoders
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encoderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Inversion logic and gains
        if (inverted) {
            outputDir = 1;
            kp = 100.00;
            ki = 10500.00;
            kd = 0.00;
            scaleFactor = -1.0 / 37.5;
        } else {
            outputDir = -1;
            kp = 250.00;
            ki = 0.00;
            kd = 5.00;
            scaleFactor = -1.0 / 100.0;
        }

        stepsPerMM = stepModes[stepMode][0];

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            int encoderCount = encoderMotor.getCurrentPosition();
            double angle = encoderCount * (360.0 / 2000.0);

            input = scaleFactor * motor.getCurrentPosition() / stepsPerMM + 200 * Math.sin(Math.toRadians(angle));
            double error = setPoint - input;

            // Basic PID calculation
            double dt = timer.seconds();
            timer.reset();

            integral += error * dt;
            double derivative = (error - lastError) / dt;

            output = kp * error + ki * integral + kd * derivative;
            lastError = error;

            // Clamp motor power (motors accept power from -1 to 1)
            double power = Math.max(-1.0, Math.min(1.0, output / 5000000.0));

            motor.setPower(outputDir * power);

            telemetry.addData("Angle", angle);
            telemetry.addData("Input", input);
            telemetry.addData("Output", output);
            telemetry.addData("Motor Power", power);
            telemetry.update();
        }
    }
}
