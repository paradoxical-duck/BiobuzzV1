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
    public static double deadbandPollen = 0;
    public static double deadbandNectar = 0;

    public static double kP_Pollen = 0.032;
    public static double kP_Nectar = 0.032;
    public static double kS_Pollen = 0.015;
    public static double kS_Nectar = 0.015;

    public static double turretLeftPollen = 0.72;
    public static double turretRightPollen = 0.28;
    public static double turretLeftNectar = 0.1283;
    public static double turretRightNectar = 0.9561;

    public static double currentPollen = Globals.turretInit;
    public static double currentNectar = Globals.turretInit;

    public static double turretMinAnglePollen = -90;
    public static double turretMaxAnglePollen = 90;
    public static double turretMinAngleNectar = -42.9;
    public static double turretMaxAngleNectar = 249.2;

    public static double turretMountOffsetPollen = 0;
    public static double turretMountOffsetNectar = 0;

    public static double requestedAnglePollen = 0;
    public static double requestedAngleNectar = 0;
    public static double targetAnglePollen = 0;
    public static double targetAngleNectar = 0;
    public static double targetServoPollen = Globals.turretInit;
    public static double targetServoNectar = Globals.turretInit;
    public static boolean reachablePollen = false;
    public static boolean reachableNectar = false;

    public static boolean upperHive = false;
    public static boolean overrideHive = false;
    public static boolean overrideUpperHive = false;
    private static boolean lastUpperHalf = false;

    public static double goalX = 0;
    public static double goalY = 0;
    public static double targetHeading = 0;
    public static double robotHeading = 0;

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
        currentPollen = robot.turret1.getPosition();
        currentNectar = robot.turret2.getPosition();
        resetSelectedHive();
    }

    public TrackingSS(Follower follower, boolean red) {
        TrackingSS.follower = follower;
        TrackingSS.alliance = red ? RobotHardware.Alliance.RED : RobotHardware.Alliance.BLUE;
        resetSelectedHive();
    }

    public void resetSelectedHive() {
        overrideHive = false;
        lastUpperHalf = follower.pose().y() > 70.75;
        overrideUpperHive = lastUpperHalf;
        setDxAndDy();
    }

    public void toggleSelectedHive() {
        setDxAndDy();

        overrideUpperHive = !upperHive;
        overrideHive = overrideUpperHive != lastUpperHalf;

        setDxAndDy();
    }

    public static void setDxAndDy() {
        double robotX = follower.pose().x();
        double robotY = follower.pose().y();

        boolean upperHalf = robotY > 70.75;

        if (upperHalf != lastUpperHalf) {
            overrideHive = false;
            lastUpperHalf = upperHalf;
        }

        upperHive = overrideHive ? overrideUpperHive : upperHalf;

        Pose goal;

        switch (alliance) {
            case RED:
                goal = upperHive ? Globals.redHiveUpSide : Globals.redHiveDownSide;
                break;
            case BLUE:
                goal = upperHive ? Globals.blueHiveUpSide : Globals.blueHiveDownSide;
                break;
            default:
                return;
        }

        goalX = goal.x();
        goalY = goal.y();

        dx = goalX - robotX;
        dy = goalY - robotY;
    }

    public static double getReachableTurretAngle(double requested, double minAngle, double maxAngle) {
        for (int i = -2; i <= 2; i++) {
            double candidate = requested + 360 * i;

            if (candidate >= minAngle && candidate <= maxAngle) {
                return candidate;
            }
        }

        double minError = Math.abs(Math.toDegrees(angleWrap(Math.toRadians(requested - minAngle))));
        double maxError = Math.abs(Math.toDegrees(angleWrap(Math.toRadians(requested - maxAngle))));

        return minError <= maxError ? minAngle : maxAngle;
    }

    public static boolean isTurretAngleReachable(double requested, double minAngle, double maxAngle) {
        for (int i = -2; i <= 2; i++) {
            double candidate = requested + 360 * i;

            if (candidate >= minAngle && candidate <= maxAngle) {
                return true;
            }
        }

        return false;
    }

    //todo ****************** TURRET TRACKING *******************
    public static double getPollenTurretPos() {
        setDxAndDy();

        targetHeading = Math.toDegrees(Math.atan2(dy, dx));
        robotHeading = Math.toDegrees(follower.pose().heading());

        requestedAnglePollen = Math.toDegrees(angleWrap(
                Math.toRadians(
                        targetHeading
                                - robotHeading
                                + turretMountOffsetPollen
                                + Globals.turretOffsetPollen
                )
        ));

        reachablePollen = isTurretAngleReachable(
                requestedAnglePollen,
                turretMinAnglePollen,
                turretMaxAnglePollen
        );

        targetAnglePollen = getReachableTurretAngle(
                requestedAnglePollen,
                turretMinAnglePollen,
                turretMaxAnglePollen
        );

        double originalServoTarget = Range.clip(
                map(
                        targetAnglePollen,
                        turretMinAnglePollen,
                        turretMaxAnglePollen,
                        turretLeftPollen,
                        turretRightPollen
                ),
                Math.min(turretLeftPollen, turretRightPollen),
                Math.max(turretLeftPollen, turretRightPollen)
        );

        targetServoPollen = turretLeftPollen + turretRightPollen - originalServoTarget;

        currentPollen = targetServoPollen;

        return currentPollen;
    }

    public static double getNectarTurretPos() {
        setDxAndDy();

        double heading = Math.toDegrees(Math.atan2(dy, dx));
        double robotAngle = Math.toDegrees(follower.pose().heading());

        requestedAngleNectar = Math.toDegrees(angleWrap(
                Math.toRadians(
                        heading
                                - robotAngle
                                + turretMountOffsetNectar
                                + Globals.turretOffsetNectar
                )
        ));

        reachableNectar = isTurretAngleReachable(
                requestedAngleNectar,
                turretMinAngleNectar,
                turretMaxAngleNectar
        );

        targetAngleNectar = getReachableTurretAngle(
                requestedAngleNectar,
                turretMinAngleNectar,
                turretMaxAngleNectar
        );

        targetServoNectar = Range.clip(
                map(
                        targetAngleNectar,
                        turretMinAngleNectar,
                        turretMaxAngleNectar,
                        turretLeftNectar,
                        turretRightNectar
                ),
                Math.min(turretLeftNectar, turretRightNectar),
                Math.max(turretLeftNectar, turretRightNectar)
        );

        currentNectar = targetServoNectar;

        return currentNectar;
    }

    public double getTurretPos() {
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

    public double getTurret() {
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

    //todo ****************** SHOOTER INTERPOLATION *******************
    public static void interpolate() {
        setDxAndDy();

        rangeVal = Math.hypot(dx, dy);

        Pose referenceHive = alliance == RobotHardware.Alliance.RED
                ? Globals.redHiveDownSide
                : Globals.blueHiveDownSide;

        double referenceX = alliance == RobotHardware.Alliance.RED
                ? shootPose.x()
                : 141.5 - shootPose.x();

        referenceDist = Math.hypot(
                referenceHive.x() - referenceX,
                referenceHive.y() - shootPose.y()
        );

        double distanceError = rangeVal - referenceDist;

        targetRpmPollen = Math.max(
                0.0,
                Globals.shooterOnPollen + distanceError * rpmPerInchPollen
        );

        targetRpmNectar = Math.max(
                0.0,
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

    public boolean trackingEnabled(ShooterSS shooterSS) {
        return shooterSS.getPollenTurretState() == ShooterSS.TurretState.TRACKING_ENABLED;
    }

    public void togglePollenTracking(ShooterSS shooterSS) {
        if (trackingEnabled(shooterSS)) {
            shooterSS.updatePollenState(ShooterSS.TurretState.SHOOT);
        }
        else {
            shooterSS.setPollenTurretTarget(getPollenTurretPos());
            shooterSS.updatePollenState(ShooterSS.TurretState.TRACKING_ENABLED);
        }
    }

    public void updatePollenTurretTracking(ShooterSS shooterSS) {
        if (trackingEnabled(shooterSS)) {
            shooterSS.setPollenTurretTarget(getPollenTurretPos());
        }
        else {
            setDxAndDy();
        }
    }

    public void updatePollenTracking(ShooterSS shooterSS) {
        currentPollen = robot.turret1.getPosition();

        shooterSS.setPollenTurretTarget(getPollenTurretPos());

        interpolate();

        shooterSS.setPollenShooterTarget(getTargetRpmPollen());
        shooterSS.setPollenHoodTarget(getTargetHoodPollen());
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