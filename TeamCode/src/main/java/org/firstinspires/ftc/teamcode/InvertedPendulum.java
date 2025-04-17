package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Inverted Pendulum", group = "")
public class InvertedPendulum extends LinearOpMode {

    private DcMotorEx motor;
    private DcMotorEx encoderMotor;
    DigitalChannel leftLimit;
    DigitalChannel rightLimit;

    double scaleFactor = 100;
    private double setPoint = 0.0;
    private double targetPosition = 0.0;

    // PID tuned for 11V battery
    private double Kp = 1.148 / scaleFactor;
    private double Ki = 0.005;
    private double Kd = 0.012 / scaleFactor;

    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;

    // Reference voltage for which PID values were tuned
    private double referenceVoltage = 10.0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        encoderMotor = hardwareMap.get(DcMotorEx.class, "encoderMotor");
        leftLimit = hardwareMap.get(DigitalChannel.class, "leftLimit");
        rightLimit = hardwareMap.get(DigitalChannel.class, "rightLimit");

        waitForStart();

        encoderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lastTime = System.currentTimeMillis();

        while (opModeIsActive() && leftLimit.getState() && rightLimit.getState()) {
            int encoderCount = encoderMotor.getCurrentPosition();
            double error = setPoint - encoderCount;

            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastTime) / 1000.0;

            integral += error * deltaTime;
            double derivative = Kd * ((error - lastError) / deltaTime);
            double proportional = Kp * error;
            double output = proportional + Ki * integral + derivative;

            lastError = error;
            lastTime = currentTime;

            // Scale the output based on the current battery voltage
            double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double voltageScaleFactor = referenceVoltage / currentVoltage;

            // Ensure the scaled output is within the range of -1.0 to 1.0
            double scaledOutput = output * voltageScaleFactor;
            double power = Math.max(-1.0, Math.min(1.0, scaledOutput));

            motor.setPower(power*voltageScaleFactor);

            telemetry.addData("Output", output);
            telemetry.addData("Scaled Output", scaledOutput);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Error", error);
            telemetry.addData("Proportional", proportional);
            telemetry.addData("Derivative", derivative);
            telemetry.addData("Encoder Value", encoderMotor.getCurrentPosition());
            telemetry.addData("Battery Voltage", currentVoltage);
            telemetry.addData("Voltage Scale Factor", voltageScaleFactor);
            telemetry.addData("Left Limit Switch", leftLimit.getState());
            telemetry.addData("Right Limit Switch", rightLimit.getState());
            telemetry.update();
        }

        motor.setPower(0);
    }
}
