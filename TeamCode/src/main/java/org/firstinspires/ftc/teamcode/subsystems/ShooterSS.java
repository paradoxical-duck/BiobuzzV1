package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/*
 * Two standalone shooters:
 *   Pollen (1) -> shooter1, turret1, hood1, feedLock1
 *   Nectar (2) -> shooter2, turret2, hood2, feedLock2
 *
 * updatePollenState(...) / updateNectarState(...) control one shooter.
 * updateState(...) / set...Target(...) without a name control both at once.
 */
public class ShooterSS {
    RobotHardware robot = RobotHardware.getInstance();
    private boolean autoS3Up = false;

    public ShooterSS(RobotHardware robot) {
        this.robot = robot;
        robot.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.applyShooterPIDF();
    }

    public void setAutoS3Up(boolean up) {
        autoS3Up = up;
    }

    //TODO BALL COLORS
    public enum ShootingMode {INIT, SHOOTING, FINAL_SHOT, RELOCK}
    private ShootingMode shootingMode = ShootingMode.INIT;

    private final ElapsedTime finishTimer = new ElapsedTime();
    public static double finishDelayMs = 100;

    private final ElapsedTime intakeLockoutTimer = new ElapsedTime();
    public static double intakeLockoutMs = 250;

    public ShootingMode getShootingMode() {
        return shootingMode;
    }

    public boolean isShooting() {
        return shootingMode == ShootingMode.SHOOTING || shootingMode == ShootingMode.FINAL_SHOT;
    }

    public boolean shooterEnabled() {
        return pollenShooterState == ShooterState.ON;
    }

    public void startShooting(IntakeSS intakeSS) {
        if (shootingMode != ShootingMode.INIT || intakeSS.getBallCount() == IntakeSS.Count.EMPTY) {
            return;
        }

        updatePollenState(ShooterState.ON);
        updateState(LockState.POLLEN_UNLOCKED);
        intakeSS.setIntakeMode(IntakeSS.IntakeMode.ON);
        shootingMode = ShootingMode.SHOOTING;
    }

    public void updateShooting(IntakeSS intakeSS) {
        IntakeSS.Count ballCount = intakeSS.getBallCount();

        if (ballCount != IntakeSS.Count.EMPTY && !shooterEnabled() && shootingMode != ShootingMode.RELOCK) {
            updatePollenState(ShooterState.ON);
        }
        else if (ballCount == IntakeSS.Count.EMPTY && shooterEnabled() && !isShooting()) {
            updatePollenState(ShooterState.INIT);
        }

        switch (shootingMode) {
            case INIT:
                break;

            case SHOOTING:
                if (ballCount == IntakeSS.Count.EMPTY) {
                    shootingMode = ShootingMode.FINAL_SHOT;
                    finishTimer.reset();
                }
                break;

            case FINAL_SHOT:
                if (ballCount != IntakeSS.Count.EMPTY) {
                    shootingMode = ShootingMode.SHOOTING;
                }
                else if (finishTimer.milliseconds() >= finishDelayMs) {
                    intakeSS.stopIntake();
                    updateState(LockState.LOCKED);
                    updatePollenState(ShooterState.INIT);

                    shootingMode = ShootingMode.RELOCK;
                    intakeLockoutTimer.reset();
                }
                break;

            case RELOCK:
                if (intakeLockoutTimer.milliseconds() >= intakeLockoutMs && getLockState() == LockState.LOCKED) {
                    shootingMode = ShootingMode.INIT;
                    intakeSS.setIntakeMode(ballCount == IntakeSS.Count.FOUR ? IntakeSS.IntakeMode.INIT : IntakeSS.IntakeMode.ON);
                }
                break;
        }
    }

    public void resetShooting(IntakeSS intakeSS) {
        shootingMode = ShootingMode.INIT;

        finishTimer.reset();
        intakeLockoutTimer.reset();

        intakeSS.stopIntake();
        intakeSS.resetBallCount();

        updateState(ShooterState.INIT);
        updateState(LockState.LOCKED);
    }

    //todo******** shooter *********
    public enum ShooterState {INIT, ON, INTERPOL_DISABLED, INTERPOLATED, AUTO, AUTO_S3}
    ShooterState pollenShooterState = ShooterState.INTERPOL_DISABLED;
    ShooterState nectarShooterState = ShooterState.INTERPOL_DISABLED;

    private double pollenShooterTarget = Globals.shooterOnPollen;
    private double nectarShooterTarget = Globals.shooterOnNectar;

    public void setPollenShooterTarget(double target) {
        pollenShooterTarget = Math.max(0, target);

        if (pollenShooterState == ShooterState.INTERPOLATED) {
            robot.shooter1.setVelocity(pollenShooterTarget);
        }
    }

    public void setNectarShooterTarget(double target) {
        nectarShooterTarget = Math.max(0, target);

        if (nectarShooterState == ShooterState.INTERPOLATED) {
            robot.shooter2.setVelocity(nectarShooterTarget);
        }
    }

    public void setShooterTarget(double target) {
        setPollenShooterTarget(target);
        setNectarShooterTarget(target);
    }

    private double shooterVelocity(ShooterState state, double interpolatedTarget, double disabledTarget, double autoTarget, double autoS3Target) {
        switch (state) {
            case INIT:
                return Globals.shooterInit;
            case ON:
            case INTERPOL_DISABLED:
                return disabledTarget;
            case INTERPOLATED:
                return interpolatedTarget;
            case AUTO:
                return autoTarget;
            case AUTO_S3:
                return autoS3Target;
        }
        return 0;
    }

    public void updatePollenState(ShooterState newState) {
        robot.shooter1.setVelocity(Math.max(0, shooterVelocity(
                newState,
                pollenShooterTarget,
                Globals.shooterOnPollen,
                Globals.autoShootPollen,
                autoS3Up ? Globals.pollenShootUp : Globals.pollenShootDown
        )));
        pollenShooterState = newState;
    }

    public void updateNectarState(ShooterState newState) {
        robot.shooter2.setVelocity(Math.max(0, shooterVelocity(
                newState,
                nectarShooterTarget,
                Globals.shooterOnNectar,
                Globals.autoShootNectar,
                Globals.shooterOnNectar
        )));
        nectarShooterState = newState;
    }

    public void updateState(ShooterState newState) {
        updatePollenState(newState);
        updateNectarState(newState);
    }

    //todo******** turret *********
    public enum TurretState {INIT, TRACKING_DISABLED, TRACKING_ENABLED, SHOOT, AUTO, AUTO_S3}
    TurretState pollenTurretState = TurretState.TRACKING_DISABLED;
    TurretState nectarTurretState = TurretState.TRACKING_DISABLED;

    private double pollenTurretTarget = Globals.turretInit;
    private double nectarTurretTarget = Globals.turretInit;

    public void setPollenTurretTarget(double target) {
        pollenTurretTarget = target;

        if (pollenTurretState == TurretState.TRACKING_ENABLED) {
            robot.turret1.setPosition(pollenTurretTarget);
        }
    }

    public void setNectarTurretTarget(double target) {
        nectarTurretTarget = target;

        if (nectarTurretState == TurretState.TRACKING_ENABLED) {
            robot.turret2.setPosition(nectarTurretTarget);
        }
    }

    public void setTurretTarget(double target) {
        setPollenTurretTarget(target);
        setNectarTurretTarget(target);
    }

    private double turretPosition(TurretState state, double trackingTarget, double currentPosition, double shootTarget, double autoTarget, double autoS3Target) {
        switch (state) {
            case INIT:
                return Globals.turretInit;
            case TRACKING_DISABLED:
                return currentPosition;
            case TRACKING_ENABLED:
                return trackingTarget;
            case SHOOT:
                return shootTarget;
            case AUTO:
                return autoTarget;
            case AUTO_S3:
                return autoS3Target;
        }
        return currentPosition;
    }

    public void updatePollenState(TurretState newState) {
        robot.turret1.setPosition(turretPosition(
                newState,
                pollenTurretTarget,
                robot.turret1.getPosition(),
                Globals.turretShootPosPollen,
                Globals.autoTurretPollen,
                autoS3Up ? Globals.pollenTurretUp : Globals.pollenTurretDown
        ));
        pollenTurretState = newState;
    }

    public void updateNectarState(TurretState newState) {
        robot.turret2.setPosition(turretPosition(
                newState,
                nectarTurretTarget,
                robot.turret2.getPosition(),
                Globals.turretShootPosNectar,
                Globals.autoTurretNectar,
                Globals.turretShootPosNectar
        ));
        nectarTurretState = newState;
    }

    public void updateState(TurretState newState) {
        updatePollenState(newState);
        updateNectarState(newState);
    }

    //todo******** hood *********
    public enum HoodState {INIT, INTERPOL_DISABLED, INTERPOLATED, SHOOT, AUTO, AUTO_S3}
    public HoodState pollenHoodState = HoodState.INTERPOL_DISABLED;
    public HoodState nectarHoodState = HoodState.INTERPOL_DISABLED;

    private double pollenHoodTarget = Globals.hoodShootPosPollen;
    private double nectarHoodTarget = Globals.hoodShootPosNectar;

    public void setPollenHoodTarget(double target) {
        pollenHoodTarget = target;

        if (pollenHoodState == HoodState.INTERPOLATED) {
            robot.hood1.setPosition(pollenHoodTarget);
        }
    }

    public void setNectarHoodTarget(double target) {
        nectarHoodTarget = target;

        if (nectarHoodState == HoodState.INTERPOLATED) {
            robot.hood2.setPosition(nectarHoodTarget);
        }
    }

    public void setHoodTarget(double target) {
        setPollenHoodTarget(target);
        setNectarHoodTarget(target);
    }

    private double hoodPosition(HoodState state, double interpolatedTarget, double currentPosition, double shootTarget, double autoTarget, double autoS3Target) {
        switch (state) {
            case INIT:
                return Globals.hoodInit;
            case INTERPOL_DISABLED:
                return currentPosition;
            case INTERPOLATED:
                return interpolatedTarget;
            case SHOOT:
                return shootTarget;
            case AUTO:
                return autoTarget;
            case AUTO_S3:
                return autoS3Target;
        }
        return currentPosition;
    }

    public void updatePollenState(HoodState newState) {
        robot.hood1.setPosition(hoodPosition(
                newState,
                pollenHoodTarget,
                robot.hood1.getPosition(),
                Globals.hoodShootPosPollen,
                Globals.autoHoodPollen,
                autoS3Up ? Globals.pollenHoodUp : Globals.pollenHoodDown
        ));
        pollenHoodState = newState;
    }

    public void updateNectarState(HoodState newState) {
        robot.hood2.setPosition(hoodPosition(
                newState,
                nectarHoodTarget,
                robot.hood2.getPosition(),
                Globals.hoodShootPosNectar,
                Globals.autoHoodNectar,
                Globals.hoodShootPosNectar
        ));
        nectarHoodState = newState;
    }

    public void updateState(HoodState newState) {
        updatePollenState(newState);
        updateNectarState(newState);
    }

    //todo ******************lock*************
    public enum LockState {UNLOCKED, LOCKED, NECTAR_UNLOCKED, POLLEN_UNLOCKED}
    public LockState lockState = LockState.UNLOCKED;

    public void updateState(LockState newState) {
        double target1 = 0; //Pollen
        double target2 = 0; //Nectar

        switch (newState) {
            case UNLOCKED:
                target1 = Globals.feedLock1Open;
                target2 = Globals.feedLock2Open;
                break;
            case LOCKED:
                target1 = Globals.feedLock1Close;
                target2 = Globals.feedLock2Close;
                break;
            case NECTAR_UNLOCKED:
                target1 = Globals.feedLock1Close;
                target2 = Globals.feedLock2Open;
                break;
            case POLLEN_UNLOCKED:
                target1 = Globals.feedLock1Open;
                target2 = Globals.feedLock2Close;
                break;
        }

        robot.feedLock1.setPosition(target1);
        robot.feedLock2.setPosition(target2);
        lockState = newState;
    }

    //todo ******************getters*************
    public ShooterState getPollenShooterState() {
        return pollenShooterState;
    }

    public ShooterState getNectarShooterState() {
        return nectarShooterState;
    }

    public HoodState getPollenHoodState() {
        return pollenHoodState;
    }

    public HoodState getNectarHoodState() {
        return nectarHoodState;
    }

    public TurretState getPollenTurretState() {
        return pollenTurretState;
    }

    public TurretState getNectarTurretState() {
        return nectarTurretState;
    }

    public LockState getLockState() {
        return lockState;
    }
}