package org.firstinspires.ftc.teamcode.ballbalancing;

public class RobotKinematics {

    private final double lp, l1, l2, lb;
    private final boolean invert;

    private final double maxh;
    private final double minh;
    private final double[] p = new double[3];
    private double h;
    private double maxtheta;

    public double[] A1 = new double[3];
    public double[] A2 = new double[3];
    public double[] A3 = new double[3];

    public double[] B1 = new double[3];
    public double[] B2 = new double[3];
    public double[] B3 = new double[3];

    public double[] C1 = new double[3];
    public double[] C2 = new double[3];
    public double[] C3 = new double[3];

    public double theta1 = 0, theta2 = 0, theta3 = 0;

    public RobotKinematics() {
        this(7.125, 11.20, 4.50, 4.00, false);
    }

    public RobotKinematics(double lp, double l1, double l2, double lb, boolean invert) {
        this.lp = lp;
        this.l1 = l1;
        this.l2 = l2;
        this.lb = lb;
        this.invert = invert;

        this.maxh = computeMaxH() - 0.2;
        this.minh = computeMinH() + 0.45;
        this.h = (maxh + minh) / 2;
        this.p[0] = 0.0;
        this.p[1] = 0.0;
        this.p[2] = maxh;

        maxTheta(h);
    }

    private double computeMaxH() {
        return Math.sqrt(Math.pow(l1 + l2, 2) - Math.pow(lp - lb, 2));
    }

    private double computeMinH() {
        if (l1 > l2) {
            return Math.sqrt(Math.pow(l1, 2) - Math.pow(lb + l2 - lp, 2));
        } else if (l2 > l1) {
            return Math.sqrt(Math.pow(l2 - l1, 2) - Math.pow(lp - lb, 2));
        } else {
            return 0;
        }
    }

    public void solveInverseKinematicsSpherical(double thetaDeg, double phiDeg, double h) {
        this.h = h;
        maxTheta(h);
        thetaDeg = Math.min(thetaDeg, maxtheta);

        double theta = Math.toRadians(thetaDeg);
        double phi = Math.toRadians(phiDeg);

        double a = Math.sin(theta) * Math.cos(phi);
        double b = Math.sin(theta) * Math.sin(phi);
        double c = Math.cos(theta);

        solveInverseKinematicsVector(a, b, c, h);
    }

    public void solveInverseKinematicsVector(double a, double b, double c, double h) {
        B1 = new double[]{-0.5 * lb, Math.sqrt(3) * 0.5 * lb, 0};
        B2 = new double[]{lb, 0, 0};
        B3 = new double[]{-0.5 * lb, -Math.sqrt(3) * 0.5 * lb, 0};

        solveTop(a, b, c, h);
        solveMiddle();

        theta1 = Math.PI / 2 - Math.atan2(Math.hypot(C1[0], C1[1]) - lb, C1[2]);
        theta2 = Math.atan2(C2[2], C2[0] - lb);
        theta3 = Math.PI / 2 - Math.atan2(Math.hypot(C3[0], C3[1]) - lb, C3[2]);
    }

    public void solveTop(double a, double b, double c, double h) {
        double sqrt3 = Math.sqrt(3);

        A1 = new double[]{
                -(lp * c) / Math.sqrt(4 * c * c + Math.pow(a - sqrt3 * b, 2)),
                (sqrt3 * lp * c) / Math.sqrt(4 * c * c + Math.pow(a - sqrt3 * b, 2)),
                h + ((a - sqrt3 * b) * lp) / Math.sqrt(4 * c * c + Math.pow(a - sqrt3 * b, 2))
        };

        A2 = new double[]{
                (lp * c) / Math.sqrt(c * c + a * a),
                0,
                h - ((lp * a) / Math.sqrt(c * c + a * a))
        };

        A3 = new double[]{
                -(lp * c) / Math.sqrt(4 * c * c + Math.pow(a + sqrt3 * b, 2)),
                -(sqrt3 * lp * c) / Math.sqrt(4 * c * c + Math.pow(a + sqrt3 * b, 2)),
                h + ((a + sqrt3 * b) * lp) / Math.sqrt(4 * c * c + Math.pow(a + sqrt3 * b, 2))
        };
    }

    public void solveMiddle() {
        double a11 = A1[0], a12 = A1[1], a13 = A1[2];
        double a21 = A2[0], a22 = A2[1], a23 = A2[2];
        double a31 = A3[0], a32 = A3[1], a33 = A3[2];

        double p1 = (-a11 + Math.sqrt(3) * a12 - 2 * lb) / a13;
        double q1 = (a11 * a11 + a12 * a12 + a13 * a13 + l2 * l2 - l1 * l1 - lb * lb) / (2 * a13);
        double r1 = p1 * p1 + 4;
        double s1 = 2 * p1 * q1 + 4 * lb;
        double t1 = q1 * q1 + lb * lb - l2 * l2;

        double p2 = (lb - a21) / a23;
        double q2 = (a21 * a21 + a23 * a23 - lb * lb + l2 * l2 - l1 * l1) / (2 * a23);
        double r2 = p2 * p2 + 1;
        double s2 = 2 * (p2 * q2 - lb);
        double t2 = q2 * q2 - l2 * l2 + lb * lb;

        double p3 = (-a31 - Math.sqrt(3) * a32 - 2 * lb) / a33;
        double q3 = (a31 * a31 + a32 * a32 + a33 * a33 + l2 * l2 - l1 * l1 - lb * lb) / (2 * a33);
        double r3 = p3 * p3 + 4;
        double s3 = 2 * p3 * q3 + 4 * lb;
        double t3 = q3 * q3 + lb * lb - l2 * l2;

        if (!invert) {
            double c11 = (-s1 - Math.sqrt(s1 * s1 - 4 * r1 * t1)) / (2 * r1);
            double c12 = -Math.sqrt(3) * c11;
            double c13 = Math.sqrt(l2 * l2 - 4 * c11 * c11 - 4 * lb * c11 - lb * lb);
            C1 = new double[]{c11, c12, c13};

            double c21 = (-s2 + Math.sqrt(s2 * s2 - 4 * r2 * t2)) / (2 * r2);
            double c22 = 0;
            double c23 = Math.sqrt(l2 * l2 - (c21 - lb) * (c21 - lb));
            C2 = new double[]{c21, c22, c23};

            double c31 = (-s3 - Math.sqrt(s3 * s3 - 4 * r3 * t3)) / (2 * r3);
            double c32 = Math.sqrt(3) * c31;
            double c33 = Math.sqrt(l2 * l2 - 4 * c31 * c31 - 4 * lb * c31 - lb * lb);
            C3 = new double[]{c31, c32, c33};
        } else {
            double c11 = (-s1 - Math.sqrt(s1 * s1 - 4 * r1 * t1)) / (2 * r1);
            double c12 = -Math.sqrt(3) * c11;
            double c13 = Math.sqrt(l2 * l2 - 4 * c11 * c11 - 4 * lb * c11 - lb * lb);
            C1 = new double[]{c11, c12, -c13};

            double c21 = (-s2 + Math.sqrt(s2 * s2 - 4 * r2 * t2)) / (2 * r2);
            double c22 = 0;
            double c23 = Math.sqrt(l2 * l2 - (c21 - lb) * (c21 - lb));
            C2 = new double[]{c21, c22, -c23};

            double c31 = (-s3 - Math.sqrt(s3 * s3 - 4 * r3 * t3)) / (2 * r3);
            double c32 = Math.sqrt(3) * c31;
            double c33 = Math.sqrt(l2 * l2 - 4 * c31 * c31 - 4 * lb * c31 - lb * lb);
            C3 = new double[]{c31, c32, -c33};
        }
    }

    public void maxTheta(double h) {
        double thetaLow = 0.0;
        double thetaHigh = Math.toRadians(20);
        double tol = 1e-3;

        while (thetaHigh - thetaLow > tol) {
            double thetaMid = (thetaLow + thetaHigh) / 2;
            if (isValidTheta(thetaMid, h)) {
                thetaLow = thetaMid;
            } else {
                thetaHigh = thetaMid;
            }
        }

        this.maxtheta = Math.max(0, Math.toDegrees(thetaLow) - 0.5);
    }

    private boolean isValidTheta(double theta, double h) {
        double c = Math.cos(theta);
        for (int s : new int[]{1, -1}) {
            double a21 = lp * c;
            double a23 = h - lp * (s * Math.sin(theta));
            try {
                double p2 = (lb - a21) / a23;
                double q2 = (a21 * a21 + a23 * a23 - lb * lb + l2 * l2 - l1 * l1) / (2 * a23);
                double r2 = p2 * p2 + 1;
                double s2 = 2 * (p2 * q2 - lb);
                double t2 = q2 * q2 - l2 * l2 + lb * lb;
                double disc = s2 * s2 - 4 * r2 * t2;
                if (disc < 0) return false;
                double c21 = (-s2 + Math.sqrt(disc)) / (2 * r2);
                double delta = l2 * l2 - (c21 - lb) * (c21 - lb);
                if (delta < 0) return false;
                double c23 = Math.sqrt(delta);
                if (Math.abs(Math.hypot(a21 - c21, a23 - c23) - l1) > 1e-3) return false;
                if (Math.abs(Math.hypot(lb - c21, c23) - l2) > 1e-3) return false;
            } catch (Exception e) {
                return false;
            }
        }
        return true;
    }

    public double getTheta1() {
        return theta1;
    }

    public double getTheta2() {
        return theta2;
    }

    public double getTheta3() {
        return theta3;
    }
}