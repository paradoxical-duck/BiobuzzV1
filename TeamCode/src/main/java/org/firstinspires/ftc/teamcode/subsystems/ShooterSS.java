
package org.firstinspires.ftc.teamcode.subsystems;

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

    public ShooterSS(RobotHardware robot) {
        this.robot = robot;
    }

    //todo******** shooter *********
    public enum ShooterState {INIT, INTERPOL_DISABLED, INTERPOLATED, AUTO}
    ShooterState pollenShooterState = ShooterState.INTERPOL_DISABLED;
    ShooterState nectarShooterState = ShooterState.INTERPOL_DISABLED;

    private double pollenShooterTarget = Globals.shooterOn;
    private double nectarShooterTarget = Globals.shooterOn;

    public void setPollenShooterTarget(double target) {
        pollenShooterTarget = target;

        if (pollenShooterState == ShooterState.INTERPOLATED) {
            robot.shooter1.setVelocity(pollenShooterTarget);
        }
    }

    public void setNectarShooterTarget(double target) {
        nectarShooterTarget = target;

        if (nectarShooterState == ShooterState.INTERPOLATED) {
            robot.shooter2.setVelocity(nectarShooterTarget);
        }
    }

    public void setShooterTarget(double target) {
        setPollenShooterTarget(target);
        setNectarShooterTarget(target);
    }

    private double shooterVelocity(ShooterState state, double interpolatedTarget) {
        switch (state) {
            case INIT:
                return Globals.shooterInit;
            case INTERPOL_DISABLED:
                return Globals.shooterOn;
            case INTERPOLATED:
                return interpolatedTarget;
            case AUTO:
                return Globals.autoShoot;
        }
        return 0;
    }

    public void updatePollenState(ShooterState newState) {
        robot.shooter1.setVelocity(shooterVelocity(newState, pollenShooterTarget));
        pollenShooterState = newState;
    }

    public void updateNectarState(ShooterState newState) {
        robot.shooter2.setVelocity(shooterVelocity(newState, nectarShooterTarget));
        nectarShooterState = newState;
    }

    public void updateState(ShooterState newState) {
        updatePollenState(newState);
        updateNectarState(newState);
    }

    //todo******** turret *********
    public enum TurretState {INIT, TRACKING_DISABLED, TRACKING_ENABLED, SHOOT, AUTO}
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

    private double turretPosition(TurretState state, double trackingTarget, double currentPosition) {
        switch (state) {
            case INIT:
                return Globals.turretInit;
            case TRACKING_DISABLED:
                return currentPosition;
            case TRACKING_ENABLED:
                return trackingTarget;
            case SHOOT:
                return Globals.turretShootPos;
            case AUTO:
                return Globals.autoTurret;
        }
        return currentPosition;
    }

    public void updatePollenState(TurretState newState) {
        robot.turret1.setPosition(turretPosition(newState, pollenTurretTarget, robot.turret1.getPosition()));
        pollenTurretState = newState;
    }

    public void updateNectarState(TurretState newState) {
        robot.turret2.setPosition(turretPosition(newState, nectarTurretTarget, robot.turret2.getPosition()));
        nectarTurretState = newState;
    }

    public void updateState(TurretState newState) {
        updatePollenState(newState);
        updateNectarState(newState);
    }

    //todo******** hood *********
    public enum HoodState {INIT, INTERPOL_DISABLED, INTERPOLATED, SHOOT, AUTO}
    public HoodState pollenHoodState = HoodState.INTERPOL_DISABLED;
    public HoodState nectarHoodState = HoodState.INTERPOL_DISABLED;

    private double pollenHoodTarget = Globals.hoodShootPos;
    private double nectarHoodTarget = Globals.hoodShootPos;

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

    private double hoodPosition(HoodState state, double interpolatedTarget, double currentPosition) {
        switch (state) {
            case INIT:
                return Globals.hoodInit;
            case INTERPOL_DISABLED:
                return currentPosition;
            case INTERPOLATED:
                return interpolatedTarget;
            case SHOOT:
                return Globals.hoodShootPos;
            case AUTO:
                return Globals.autoHood;
        }
        return currentPosition;
    }

    public void updatePollenState(HoodState newState) {
        robot.hood1.setPosition(hoodPosition(newState, pollenHoodTarget, robot.hood1.getPosition()));
        pollenHoodState = newState;
    }

    public void updateNectarState(HoodState newState) {
        robot.hood2.setPosition(hoodPosition(newState, nectarHoodTarget, robot.hood2.getPosition()));
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
                target1 = Globals.unlocked;
                target2 = Globals.unlocked;
                break;
            case LOCKED:
                target1 = Globals.locked;
                target2 = Globals.locked;
                break;
            case NECTAR_UNLOCKED:
                target1 = Globals.locked;
                target2 = Globals.unlocked;
                break;
            case POLLEN_UNLOCKED:
                target1 = Globals.unlocked;
                target2 = Globals.locked;
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
