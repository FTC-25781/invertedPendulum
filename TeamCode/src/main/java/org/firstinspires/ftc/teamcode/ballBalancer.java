package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "PID Distance Servo Control")
public class ballBalancer extends LinearOpMode {

    private DistanceSensor distanceSensor;
    private Servo beamServo;
    private DigitalChannel limitSwitch;

    static double minPositionForServo = 0.36;

    static double maxPositionForServo = 0.66;

    private double defaultPosition = 0.50;

    private double increment = 0.01;

    // PID coefficients
    private double kP = 0.001;
    private double kI = 0.0;
    private double kD = 0.0;

    private double targetDistance = 10.0;
    private double previousError = 0;
    private double integral = 0;

    private double servoPosition = 0.5;

    @Override
    public void runOpMode(){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        beamServo = hardwareMap.get(Servo.class, "beamServo");
        /*
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit_switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        */

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
//            if (!limitSwitch.getState()) {
//                telemetry.addData("Safety", "Limit Switch Activated - Stopping the System");
//                telemetry.update();
//                stop();
//                return;
//            }
            // PID Part
            double currentDistance = distanceSensor.getDistance(DistanceUnit.CM);
            double error = targetDistance - currentDistance;
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
