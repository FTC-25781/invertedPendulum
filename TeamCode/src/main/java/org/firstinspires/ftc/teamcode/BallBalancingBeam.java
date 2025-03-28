package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Ball Balance Beam", group = "Examples")
public class BallBalancingBeam extends LinearOpMode {

    private Servo beamServo;
    private DistanceSensor ballSensor;

    // PID constants
    private double kP = 0.05;
    private double kI = 0;
    private double kD = 0.02;

    private double targetDistance = 15.0; // Target ball distance in cm (centered)
    private double integral = 0;
    private double lastError = 0;

    @Override
    public void runOpMode() {

        // Hardware setup
        beamServo = hardwareMap.get(Servo.class, "beamServo");
        ballSensor = hardwareMap.get(DistanceSensor.class, "ballSensor");

        beamServo.setPosition(0.5); // Neutral

        waitForStart();

        while (opModeIsActive()) {
            double currentDistance = ballSensor.getDistance(DistanceUnit.CM);

            // Compute PID
            double error = targetDistance - currentDistance;
            integral += error;
            double derivative = error - lastError;
            double output = (kP * error) + (kI * integral) + (kD * derivative);

            // Clamp and map servo output
            double beamServoPosition = 0.5 + output;
            beamServoPosition = Math.max(0.0, Math.min(1.0, beamServoPosition)); // clamp between 0 and 1
            beamServo.setPosition(beamServoPosition);

            telemetry.addData("Distance (cm)", currentDistance);
            telemetry.addData("Error", error);
            telemetry.addData("Servo Pos", beamServoPosition);
            telemetry.update();

            lastError = error;

            sleep(20); // ~50 Hz control loop
        }
    }
}
