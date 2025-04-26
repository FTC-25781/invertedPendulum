package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "PID Distance Servo Control")
public class ballBalancer extends LinearOpMode {

    private DistanceSensor distanceSensor;
    private Servo beamServo;
    static double minPositionForServo = 0.36;

    static double maxPositionForServo = 0.66;

    private double defaultPosition = 0.50;

    private double increment = 0.01;

    // PID coefficients
    private double kP = 0.00001; // 0.001
    private double kI = 0.0;
    private double kD = 0.000;

    private double targetDistance = 15.0;
    private double previousError = 0;
    private double integral = 0;

    private double servoPosition = 0.5;

    @Override
    public void runOpMode(){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        beamServo = hardwareMap.get(Servo.class, "beamServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            // PID Part
            double currentDistance = distanceSensor.getDistance(DistanceUnit.CM);
            double error = targetDistance - currentDistance;
            if(Math.abs(error) < 2) {
                error = 0;
            }
            integral += error;
            double derivative = error - previousError;
            previousError = error;

            double outputValue = (kP * error) + (kI * integral) + (kD * derivative);

            // Servo range [0,1]
            servoPosition = Math.max(minPositionForServo, Math.min(maxPositionForServo, servoPosition - outputValue)); //0.5 is default position for a servo
            beamServo.setPosition(servoPosition);

            telemetry.addData("Distance", currentDistance);
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Output Value", outputValue);
            telemetry.update();
        }
    }
}
