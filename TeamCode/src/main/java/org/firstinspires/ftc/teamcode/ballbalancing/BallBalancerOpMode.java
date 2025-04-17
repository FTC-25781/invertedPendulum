package org.firstinspires.ftc.teamcode.ballbalancing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Ball Balancer Test", group = "Robot")
public class BallBalancerOpMode extends LinearOpMode {

    private platformController controller;

    @Override
    public void runOpMode() {
        // Create and link kinematics + controller
        RobotKinematics model = new RobotKinematics();
        controller = new platformController(hardwareMap, model, telemetry);

        telemetry.addLine("Ball Balancing Robot Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Demo move - static target position
        controller.gotoTimeSpherical(10, 45, 8.2, 0.5);
        sleep(1000);

        // Run a sample dance
        controller.dance1();
        while (opModeIsActive()) {
//            double y = Math.abs(gamepad1.left_stick_y);
//            double x = Math.abs(gamepad1.left_stick_x);
//            double z = Math.abs(gamepad1.right_stick_y);
//            controller.gotoInstantVector(x, y, z, 8);
            telemetry.addLine("Press stop to end.");
            telemetry.update();
            sleep(10);
        }

        controller.closeLog();
    }
}
