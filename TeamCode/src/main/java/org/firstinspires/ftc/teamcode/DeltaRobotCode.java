package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class DeltaRobotCode extends OpMode {

    private Servo servoX;
    private Servo servoY;
    private Servo servoZ;

    private PIDController pidX, pidY, pidZ;

    private double targetDistanceX = 0;
    private double currentDistanceX = 0;
    private double targetDistanceY = 0;
    private double currentDistanceY = 0;
    private double targetDistanceZ = 0;
    private double currentDistanceZ = 0;

    public void init(){

        servoX = hardwareMap.get(Servo.class, "ServoX");
        servoY = hardwareMap.get(Servo.class, "ServoY");
        servoZ = hardwareMap.get(Servo.class, "ServoZ");

        pidX = new PIDController(0.05, 0.005, 0.01);
        pidY = new PIDController(0.05, 0.005, 0.01);
        pidZ = new PIDController(0.05, 0.005, 0.01);
    }

    public void loop(){

        targetDistanceX = gamepad1.left_stick_x;
        targetDistanceY = gamepad1.left_stick_y;
        targetDistanceZ = gamepad1.right_trigger;

        double outputX = pidX.PIDCalculation(targetDistanceX, currentDistanceX);
        double outputY = pidX.PIDCalculation(targetDistanceY, currentDistanceY);
        double outputZ = pidX.PIDCalculation(targetDistanceZ, currentDistanceZ);

        currentDistanceX += outputX;
        currentDistanceY += outputY;
        currentDistanceZ += outputZ;

        servoX.setPosition(currentDistanceX);
        servoY.setPosition(currentDistanceY);
        servoZ.setPosition(currentDistanceZ);

        telemetry.addData("Target X", targetDistanceX);
        telemetry.addData("Current X", currentDistanceX);
        telemetry.addData("Target Y", targetDistanceY);
        telemetry.addData("Current Y", currentDistanceY);
        telemetry.addData("Target Z", targetDistanceZ);
        telemetry.addData("Current Z", currentDistanceZ);
        telemetry.update();
    }
    @Override
    public void stop(){ // to reset PID controller at the end of the TeleOp
        pidX.reset();
        pidY.reset();
        pidZ.reset();
    }
    public class PIDController {
        private double kP;
        private double kI;
        private double kD;
        private double previousError;
        private double integral;
        private double output;

        public PIDController(double kP, double kD, double kI){

            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.previousError = 0;
            this.integral = 0;
        }
        public double PIDCalculation(double setPoint, double currentValue){

            double error = setPoint - currentValue;
            integral += error;
            double derivative = error - previousError;

            output = kP * error + kI * integral + kD * derivative;
            previousError = error;

            return output; // because return type in method is specified as double so need to return a double
        }
        public void reset(){
            previousError = 0;
            integral = 0;
        }
    }
}
