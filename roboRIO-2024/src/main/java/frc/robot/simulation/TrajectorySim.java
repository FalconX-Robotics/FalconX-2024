package frc.robot.simulation;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.util.Units;
import frc.robot.DashboardHelper;
import frc.robot.DashboardHelper.LogLevel;

public class TrajectorySim {
public static final double speakerHeight = Units.inchesToMeters(80.5); // meters
    public static final double g = 9.8; // acceleration due to gravity m/s^2

    //meters
    public static double getAngle(double x, double y) {
        return Math.atan(speakerHeight/x);
        // double vy; // Y-Velocity on shoot
        // double vx; // X-Velocity on shoot
        // double tyMax; // Time it takes for note to reach peak

        // vy = Math.sqrt((speakerHeight - y)*2*g);
        // tyMax = vy/g;
        // vx = x/tyMax;
        // DashboardHelper.putNumber(LogLevel.Debug, "vx", vx);
        // DashboardHelper.putNumber(LogLevel.Debug, "vy", vy);
        // return Math.atan(vy/vx);
    } //radians

    public static double getVelocity(double x, double y) {
        return 3000;
        // double vy;
        // double vx;
        // double tyMax;

        // vy = Math.sqrt((speakerHeight - y)*2*g);
        // tyMax = vy/g;
        // vx = -x/tyMax;
        // return Math.sqrt(Math.pow(vy, 2) + Math.pow(vx, 2));
    }

    public static double convertMetersToRPM(double meters) {
        double wheelDiameter = Units.inchesToMeters(4);
        double wheelCircumference = wheelDiameter * Math.PI;

        double mpm = meters * 60;

        return 3000;
        // return mpm / wheelCircumference;
    }
}
