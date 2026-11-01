
package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ShooterSS {
    RobotHardware robot = RobotHardware.getInstance();

    public ShooterSS(RobotHardware robot) {
        this.robot = robot;
    }

    //todo******** shooter *********
    public enum ShooterState {INIT, INTERPOL_DISABLED, INTERPOLATED, AUTO}
    ShooterState shooterState = ShooterState.INTERPOL_DISABLED;

    private double shooterTarget = Globals.shooterOn;

    public void setShooterTarget(double target) {
        shooterTarget = target;

        if (shooterState == ShooterState.INTERPOLATED) {
            robot.shooter1.setVelocity(shooterTarget);
            robot.shooter2.setVelocity(shooterTarget);
        }
    }

    public void updateState(ShooterState newState) {
        double target = 0;

        switch (newState) {
            case INIT:
                target = Globals.shooterInit;
                break;
            case INTERPOL_DISABLED:
                target = Globals.shooterOn;
                break;
            case INTERPOLATED:
                target = shooterTarget;
                break;
            case AUTO:
                target = Globals.autoShoot;
                break;
        }

        robot.shooter1.setVelocity(target);
        robot.shooter2.setVelocity(target);
        shooterState = newState;
    }

    //todo******** turret *********
    public enum TurretState {INIT, TRACKING_DISABLED, TRACKING_ENABLED, SHOOT, AUTO}
    TurretState turretState = TurretState.TRACKING_DISABLED;

    private double turretTarget = Globals.turretInit;

    public void setTurretTarget(double target) {
        turretTarget = target;

        if (turretState == TurretState.TRACKING_ENABLED) {
            robot.turret.setPosition(turretTarget);
        }
    }

    public void updateState(TurretState newState) {
        double target = robot.turret.getPosition();

        switch (newState) {
            case INIT:
                target = Globals.turretInit;
                break;
            case TRACKING_DISABLED:
                break;
            case TRACKING_ENABLED:
                target = turretTarget;
                break;
            case SHOOT:
                target = Globals.turretShootPos;
                break;
            case AUTO:
                target = Globals.autoTurret;
                break;
        }

        robot.turret.setPosition(target);
        turretState = newState;
    }

    //todo******** hood *********
    public enum HoodState {INIT, INTERPOL_DISABLED, INTERPOLATED, SHOOT, AUTO}
    public HoodState hoodState = HoodState.INTERPOL_DISABLED;

    private double hoodTarget = Globals.hoodShootPos;

    public void setHoodTarget(double target) {
        hoodTarget = target;

        if (hoodState == HoodState.INTERPOLATED) {
            robot.hood.setPosition(hoodTarget);
        }
    }

    public void updateState(HoodState newState) {
        double target = robot.hood.getPosition();

        switch (newState) {
            case INIT:
                target = Globals.hoodInit;
                break;
            case INTERPOL_DISABLED:
                break;
            case INTERPOLATED:
                target = hoodTarget;
                break;
            case SHOOT:
                target = Globals.hoodShootPos;
                break;
            case AUTO:
                target = Globals.autoHood;
                break;
        }

        robot.hood.setPosition(target);
        hoodState = newState;
    }

    //todo ******************lock*************
    public enum LockState {UNLOCKED, LOCKED, NECTAR_UNLOCKED, POLLEN_UNLOCKED}
    public LockState lockState = LockState.UNLOCKED;

    public void updateState(LockState newState) {
        double target1 = 0;
        double target2 = 0;

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

        robot.lock1.setPosition(target1);
        robot.lock2.setPosition(target2);
        lockState = newState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public HoodState getHoodState() {
        return hoodState;
    }

    public TurretState getTurretState() {
        return turretState;
    }

    public LockState getLockState() {
        return lockState;
    }
}
