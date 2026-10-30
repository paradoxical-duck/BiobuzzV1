
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Side;

public class TrackingSS {
    public static RobotHardware robot = RobotHardware.getInstance();
    public static Follower follower;
    public static Side.ALLIANCE alliance;
    public static double dy = 0;
    public static double dx = 0;

    // todo turret tracking constants
    public static double deadband = 2.0;
    public static double kP = 0.032;
    public static double kS = 0.015;
    public static double turretLeft = 0.1283, turretRight = 0.9561; //todo tune required
    public static double current = 0.5;

    //todo shooter and hood interpolation constants
    public static Pose shootPose = new Pose(72, 40, 0);
    public static double rpmPerInch = 0;
    public static double hoodPerInch = 0;

    public static double targetRpm = Globals.shooterOn;
    public static double targetHood = Globals.hoodShootPos;
    public static double rangeVal = 0;
    public static double referenceDist = 0;

    public TrackingSS(RobotHardware robot, Follower follower, Side.ALLIANCE alliance) {
        this.robot = robot;
        this.follower = follower;
        this.alliance = alliance;
    }

    public static void setDxAndDy() {
        double robotX = follower.pose().x();
        double robotY = follower.pose().y();

        switch (alliance) {
            case RED:
                if (robotY > 70.75) /*on upper half of field */ {
                    dx = Globals.redHiveUpSide.x() - robotX;
                    dy = Globals.redHiveUpSide.y() - robotY;
                }
                else {
                    dx = Globals.redHiveDownSide.x() - robotX;
                    dy = Globals.redHiveDownSide.y() - robotY;
                }
                break;
            case BLUE:
                if (robotY > 70.75) /*on upper half of field */ {
                    dx = Globals.blueHiveUpSide.x() - robotX;
                    dy = Globals.blueHiveUpSide.y() - robotY;
                }
                else {
                    dx = Globals.blueHiveDownSide.x() - robotX;
                    dy = Globals.blueHiveDownSide.y() - robotY;
                }
                break;
        }
    }

    //todo ***************** TURRET TRACKING ************************
    public static double getTurretPos() {
        setDxAndDy();

        double targetHeading = Math.atan2(dy, dx); // arctan function using perpendicular and base to find theta :)
        double robotHeading = follower.pose().heading();

        double turretTarget = angleWrap(targetHeading - robotHeading);
        double turretCurrent = Math.toRadians(getTurret());

        double error = angleWrap(turretTarget - turretCurrent);

        if (Math.abs(Math.toDegrees(error)) > deadband) {
            double adjustment = error * kP;
            if (Math.abs(Math.toDegrees(error)) > deadband + 1.5) {
                adjustment += Math.signum(error) * kS;
            }
            current = Range.clip(current + adjustment, turretLeft, turretRight);
        }
        return current;
    }

    public static double getTurret() {
        return map(current, turretLeft, turretRight, -42.9, 249.2);
    }

    public static double getTurret(RobotHardware robot) {
        return robot.turret.getPosition();
    }

    public static double map(double val, double inMin, double inMax, double outMin, double outMax) {
        return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static double angleWrap(double error) {
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return error;
    }

    //todo ***************** SHOOTER INTERPOLATION ************************
    public static void interpolate() {
        setDxAndDy();

        double robotX = follower.pose().x();
        double robotY = follower.pose().y();

        double goalX = robotX + dx;
        double goalY = robotY + dy;

        rangeVal = Math.hypot(dx, dy);

        referenceDist = Math.hypot(
                goalX - shootPose.x(),
                goalY - shootPose.y()
        );

        double distanceError = rangeVal - referenceDist;

        targetRpm = Math.max(0,
                Globals.shooterOn + distanceError * rpmPerInch);

        targetHood = Range.clip(
                Globals.hoodShootPos + distanceError * hoodPerInch,
                0.0, 1.0);
    }

    public static double getTargetRpm() {
        return targetRpm;
    }

    public static double getTargetHood() {
        return targetHood;
    }

    public static void updateTracking(ShooterSS shooterSS) {
        shooterSS.setTurretTarget(getTurretPos());
        interpolate();
        shooterSS.setShooterTarget(getTargetRpm());
        shooterSS.setHoodTarget(getTargetHood());
    }
}
