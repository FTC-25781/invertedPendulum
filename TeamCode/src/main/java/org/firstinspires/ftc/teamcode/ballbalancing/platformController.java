package org.firstinspires.ftc.teamcode.ballbalancing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class platformController {
    private final RobotKinematics robot;
    CsvLogger logger = new CsvLogger("robot_log.csv");
    private final Telemetry telemetry;
    private final Servo s1, s2, s3;

    public platformController(HardwareMap hardwareMap, RobotKinematics model, Telemetry telemetry) {
        this.robot = model;
        this.telemetry = telemetry;
        this.s1 = hardwareMap.get(Servo.class, "servoX");
        this.s2 = hardwareMap.get(Servo.class, "servoY");
        this.s3 = hardwareMap.get(Servo.class, "servoZ");
        initialize();
    }

    private double clamp(double value, double lower, double upper) {
        return Math.max(lower, Math.min(value, upper));
    }

    private void initialize() {
        telemetryPrint("Initializing...");
        setMotorAngles(54, 54, 54);
        interpolateTime(new double[]{19, 19, 19}, 0.25);
        sleep(1000);
        gotoTimeSpherical(0, 0, 8.26, 0.25);
        sleep(1000);
        telemetryPrint("Initialized!");
    }

    public void setMotorAngles(double t1, double t2, double t3) {
        //telemetryPrint("setting motor angles: " + t1 + ", " + t2 + ", " + t3);
//        telemetry.addData("Setting motor Position: ","x=%.3f, y=%.3f, z=%.3f",
//                mapAngleToServo(clamp(t1, 19, 90)),
//                mapAngleToServo(clamp(t2, 19, 90)) ,
//                mapAngleToServo(clamp(t3, 19, 90)));
        s1.setPosition(mapAngleToServo(clamp(t1, 19, 90)));
        s2.setPosition(mapAngleToServo(clamp(t2, 19, 90)));
        s3.setPosition(mapAngleToServo(clamp(t3, 19, 90)));
    }

    private double mapAngleToServo(double angle) {
        return ((angle/180) -0.3);
    }

    private double unmapServoToAngle(double position) {
        return 19 + position * (90 - 19);
    }

    public void interpolateTime(double[] targetAngles, double duration) {
        double[] current = {
                unmapServoToAngle(s1.getPosition()),
                unmapServoToAngle(s2.getPosition()),
                unmapServoToAngle(s3.getPosition())
        };
        int steps = Math.max(1, (int) (duration / 0.01));
        for (int i = 0; i <= steps; i++) {
            double t = i * duration / steps;
            double[] angles = new double[3];
            for (int j = 0; j < 3; j++) {
                angles[j] = current[j] + (targetAngles[j] - current[j]) * t / duration;
            }
            setMotorAngles(angles[0], angles[1], angles[2]);
            sleep((int) (duration * 1000 / steps));
        }
    }

    public void interpolateSpeed(double[] targetAngles, double speed) {
        double[] current = {
                unmapServoToAngle(s1.getPosition()),
                unmapServoToAngle(s2.getPosition()),
                unmapServoToAngle(s3.getPosition())
        };
        double[] durations = new double[3];
        for (int i = 0; i < 3; i++) {
            durations[i] = Math.abs(targetAngles[i] - current[i]) / speed;
        }
        double maxDuration = Math.max(durations[0], Math.max(durations[1], durations[2]));
        int steps = Math.max(1, (int) (maxDuration / 0.01));
        for (int i = 0; i <= steps; i++) {
            double t = i * maxDuration / steps;
            double[] angles = new double[3];
            for (int j = 0; j < 3; j++) {
                angles[j] = current[j] + (targetAngles[j] - current[j]) * Math.min(t / durations[j], 1);
            }
            setMotorAngles(angles[0], angles[1], angles[2]);
            sleep((int) (maxDuration * 1000 / steps));
        }
    }

    public void gotoTimeSpherical(double theta, double phi, double h, double t) {
        robot.solveInverseKinematicsSpherical(theta, phi, h);
        double[] targetAngles = {
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta1()),
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta2()),
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta3())
        };
        interpolateTime(targetAngles, t);
    }

    public void gotoTimeVector(double a, double b, double c, double h, double t) {
        robot.solveInverseKinematicsVector(a, b, c, h);
        double[] targetAngles = {
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta1()),
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta2()),
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta3())
        };
        interpolateTime(targetAngles, t);
    }

    public void gotoInstantVector(double a, double b, double c, double h) {
        robot.solveInverseKinematicsVector(a, b, c, h);
        double[] targetAngles = {
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta1()),
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta2()),
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta3())
        };
        telemetry.addData("gotoInstantVector ","Theta1=%.3f, Theta2=%.3f, Theta3=%.3f",
                robot.getTheta1(), robot.getTheta2(), robot.getTheta3());
        telemetry.addData("gotoInstantVector ","targetAngles0=%.3f, targetAngles1=%.3f, targetAngles2=%.3f",
                targetAngles[0], targetAngles[1], targetAngles[2]);
        setMotorAngles(targetAngles[0], targetAngles[1], targetAngles[2]);
    }

    public void gotoInstantSpherical(double theta, double phi, double h) {
        robot.solveInverseKinematicsSpherical(theta, phi, h);
        double[] targetAngles = {
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta1()),
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta2()),
                Math.toDegrees(Math.PI * 0.5 - robot.getTheta3())
        };
        setMotorAngles(targetAngles[0], targetAngles[1], targetAngles[2]);
    }

    public void dance1() {
        gotoTimeVector(0.258819045103, 0, 0.965925826289, 8, 1.0);
        for (int loop = 0; loop < 3; loop++) {
            for (int i = 0; i < 100; i++) {
                double t = (2 * Math.PI / 100) * i;
                double x = Math.cos(Math.PI * 5 / 12) * Math.cos(t);
                double y = Math.cos(Math.PI * 5 / 12) * Math.sin(t);
                double z = Math.sin(Math.PI * 5 / 12);
                gotoInstantVector(x, y, z, 8);
                telemetry.addData("Dance-1 ","i=%d, t=%.3f, x=%.3f, y=%.3f, z=%.3f", i, t, x, y, z);
                telemetry.addLine("servo 0 position: " + s1.getPosition());
                telemetry.addLine("servo 1 position: " + s2.getPosition());
                telemetry.addLine("servo 2 position: " + s3.getPosition());
                logger.log(String.format("%d, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
                                          i,    t,     x,    y,    z,
                        s1.getPosition(), s2.getPosition(), s3.getPosition()));
                telemetry.update();
                sleep(10);
            }
        }
        gotoTimeVector(0, 0, 1, 8, 1.0);
    }

    private void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ignored) {
        }
    }

    private void telemetryPrint(String msg) {
        telemetry.addLine((msg));
        telemetry.update();
    }

    public void update() {

        telemetry.update();
    }

    public void closeLog() {
        logger.close();
    }
}
