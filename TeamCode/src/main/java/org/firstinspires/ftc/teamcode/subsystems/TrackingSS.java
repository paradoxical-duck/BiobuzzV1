package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class TrackingSS {
    private static RobotHardware robot;
    private static Follower follower;
    public static RobotHardware.Alliance alliance;
    public static double dy = 0;
    public static double dx = 0;

    // todo turret tracking constants
    public static double deadbandPollen = 2.0;
    public static double deadbandNectar = 2.0;

    public static double kP_Pollen = 0.032;
    public static double kP_Nectar = 0.032;
    public static double kS_Pollen = 0.015;
    public static double kS_Nectar = 0.015;

    public static double turretLeftPollen = 0.1283;
    public static double turretRightPollen = 0.9561;
    public static double turretLeftNectar = 0.1283;
    public static double turretRightNectar = 0.9561;

    public static double currentPollen = Globals.turretInit;
    public static double currentNectar = Globals.turretInit;

    public static double turretMinAnglePollen = -42.9;
    public static double turretMaxAnglePollen = 249.2;
    public static double turretMinAngleNectar = -42.9;
    public static double turretMaxAngleNectar = 249.2;

    //todo shooter and hood interpolation constants
    public static Pose shootPose = new Pose(72, 40, 0);

    public static double rpmPerInchPollen = 0;
    public static double rpmPerInchNectar = 0;
    public static double hoodPerInchPollen = 0;
    public static double hoodPerInchNectar = 0;

    public static double targetRpmPollen = Globals.shooterOnPollen;
    public static double targetRpmNectar = Globals.shooterOnNectar;
    public static double targetHoodPollen = Globals.hoodShootPosPollen;
    public static double targetHoodNectar = Globals.hoodShootPosNectar;

    public static double rangeVal = 0;
    public static double referenceDist = 0;

    public TrackingSS(RobotHardware robot, Follower follower, RobotHardware.Alliance alliance) {
        TrackingSS.robot = robot;
        TrackingSS.follower = follower;
        TrackingSS.alliance = alliance;
    }

    public TrackingSS(Follower follower, boolean red) {
        TrackingSS.follower = follower;
        TrackingSS.alliance = red ? RobotHardware.Alliance.RED : RobotHardware.Alliance.BLUE;
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
    public static double getPollenTurretPos() {
        setDxAndDy();

        double targetHeading = Math.atan2(dy, dx);
        double robotHeading = follower.pose().heading();

        double turretTarget = angleWrap(
                targetHeading - robotHeading + Math.toRadians(Globals.turretOffsetPollen)
        );

        double turretCurrent = Math.toRadians(getPollenTurret());
        double error = angleWrap(turretTarget - turretCurrent);

        if (Math.abs(Math.toDegrees(error)) > deadbandPollen) {
            double adjustment = error * kP_Pollen;

            if (Math.abs(Math.toDegrees(error)) > deadbandPollen + 1.5) {
                adjustment += Math.signum(error) * kS_Pollen;
            }

            currentPollen = Range.clip(
                    currentPollen + adjustment,
                    turretLeftPollen,
                    turretRightPollen
            );
        }

        return currentPollen;
    }

    public static double getNectarTurretPos() {
        setDxAndDy();

        double targetHeading = Math.atan2(dy, dx);
        double robotHeading = follower.pose().heading();

        double turretTarget = angleWrap(
                targetHeading - robotHeading + Math.toRadians(Globals.turretOffsetNectar)
        );

        double turretCurrent = Math.toRadians(getNectarTurret());
        double error = angleWrap(turretTarget - turretCurrent);

        if (Math.abs(Math.toDegrees(error)) > deadbandNectar) {
            double adjustment = error * kP_Nectar;

            if (Math.abs(Math.toDegrees(error)) > deadbandNectar + 1.5) {
                adjustment += Math.signum(error) * kS_Nectar;
            }

            currentNectar = Range.clip(
                    currentNectar + adjustment,
                    turretLeftNectar,
                    turretRightNectar
            );
        }

        return currentNectar;
    }

    public static double getTurretPos() {
        return getPollenTurretPos();
    }

    public static double getPollenTurret() {
        return map(
                currentPollen,
                turretLeftPollen,
                turretRightPollen,
                turretMinAnglePollen,
                turretMaxAnglePollen
        );
    }

    public static double getNectarTurret() {
        return map(
                currentNectar,
                turretLeftNectar,
                turretRightNectar,
                turretMinAngleNectar,
                turretMaxAngleNectar
        );
    }

    public static double getTurret() {
        return getPollenTurret();
    }

    public static double map(double val, double inMin, double inMax, double outMin, double outMax) {
        return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static double angleWrap(double error) {
        while (error > Math.PI) {
            error -= 2 * Math.PI;
        }
        while (error < -Math.PI) {
            error += 2 * Math.PI;
        }
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

        targetRpmPollen = Math.max(
                0,
                Globals.shooterOnPollen + distanceError * rpmPerInchPollen
        );

        targetRpmNectar = Math.max(
                0,
                Globals.shooterOnNectar + distanceError * rpmPerInchNectar
        );

        targetHoodPollen = Range.clip(
                Globals.hoodShootPosPollen + distanceError * hoodPerInchPollen,
                0.0,
                1.0
        );

        targetHoodNectar = Range.clip(
                Globals.hoodShootPosNectar + distanceError * hoodPerInchNectar,
                0.0,
                1.0
        );
    }

    public static double getTargetRpmPollen() {
        return targetRpmPollen;
    }

    public static double getTargetRpmNectar() {
        return targetRpmNectar;
    }

    public static double getTargetHoodPollen() {
        return targetHoodPollen;
    }

    public static double getTargetHoodNectar() {
        return targetHoodNectar;
    }

    public static double getTargetRpm() {
        return targetRpmPollen;
    }

    public static double getTargetHood() {
        return targetHoodPollen;
    }

    public static void updateTracking(ShooterSS shooterSS) {
        currentPollen = robot.turret1.getPosition();
        currentNectar = robot.turret2.getPosition();
        shooterSS.setPollenTurretTarget(getPollenTurretPos());
        shooterSS.setNectarTurretTarget(getNectarTurretPos());

        interpolate();

        shooterSS.setPollenShooterTarget(getTargetRpmPollen());
        shooterSS.setNectarShooterTarget(getTargetRpmNectar());

        shooterSS.setPollenHoodTarget(getTargetHoodPollen());
        shooterSS.setNectarHoodTarget(getTargetHoodNectar());
    }
}