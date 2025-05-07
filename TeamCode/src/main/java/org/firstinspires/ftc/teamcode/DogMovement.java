package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Dog Movement", group="Autonomous")
public class DogMovement extends LinearOpMode {

    private Servo frontLeft, frontRight, backLeft, backRight;

    // sweep endpoints
    private static final double POS_DOWN    = 0.2;
    private static final double POS_UP      = 0.8;
    // how long to hold each diagonal (in seconds)
    private static final double INTERVAL_SEC = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft  = hardwareMap.get(Servo.class, "frontLeftServo");
        frontRight = hardwareMap.get(Servo.class, "frontRightServo");
        backLeft   = hardwareMap.get(Servo.class, "backLeftServo");
        backRight  = hardwareMap.get(Servo.class, "backRightServo");

        // set servo directions
        frontLeft .setDirection(Servo.Direction.FORWARD);
        backRight .setDirection(Servo.Direction.FORWARD);
        frontRight.setDirection(Servo.Direction.REVERSE);
        backLeft  .setDirection(Servo.Direction.REVERSE);

        // start all legs down
        frontLeft .setPosition(POS_DOWN);
        frontRight.setPosition(POS_DOWN);
        backLeft  .setPosition(POS_DOWN);
        backRight .setPosition(POS_DOWN);

        telemetry.addLine("Ready to Move");
        telemetry.update();
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        boolean diag1Up = true;   // start with front‐left + back‐right

        while (opModeIsActive()) {
            if (timer.seconds() >= INTERVAL_SEC) {
                if (diag1Up) {
                    // FL + BR go up, the other two go down
                    frontLeft .setPosition(POS_UP);
                    backRight .setPosition(POS_UP);
                    frontRight.setPosition(POS_DOWN);
                    backLeft  .setPosition(POS_DOWN);
                } else {
                    // FR + BL go up, the other two go down
                    frontRight.setPosition(POS_UP);
                    backLeft  .setPosition(POS_UP);
                    frontLeft .setPosition(POS_DOWN);
                    backRight .setPosition(POS_DOWN);
                }
                diag1Up = !diag1Up;    // flip for next cycle
                timer.reset();
            }

            //telemetry showing current state
            telemetry
                    .addData("Next flip in", "%.1f sec", INTERVAL_SEC - timer.seconds())
                    .addData("Current diagonal", diag1Up ? "FL + BR" : "FR + BL");
            telemetry.update();
        }
    }
}
