
package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ShooterSS {
    RobotHardware robot = RobotHardware.getInstance();

    public ShooterSS(RobotHardware robot) {
        this.robot = robot;
    }

    //todo******** shooter *********
    public enum ShooterState {INIT, INTERPOL_DISABLED, INTERPOLATED,}
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
        }
        robot.shooter1.setVelocity(target);
        robot.shooter2.setVelocity(target);
        shooterState = newState;
    }

    //todo******** turret *********
    public enum TurretState {INIT, TRACKING_DISABLED, TRACKING_ENABLED, SHOOT}
    TurretState turretState = TurretState.TRACKING_DISABLED;

    private double turretTarget = Globals.turretInit;

    public void setTurretTarget(double target) {
        turretTarget = target;

        if (turretState == TurretState.TRACKING_ENABLED) {
            robot.turret.setPosition(turretTarget);
        }
    }

    public void updateState(TurretState newState) {
        turretState = newState;
        switch (newState) {
            case TRACKING_DISABLED:
                break;
            case INIT:
                robot.turret.setPosition(Globals.turretInit);
                break;
            case TRACKING_ENABLED:
                robot.turret.setPosition(turretTarget);
                break;
            case SHOOT:
                robot.turret.setPosition(Globals.turretShootPos);
                break;
        }
    }

    //todo******** hood *********
    public enum HoodState {INIT, INTERPOL_DISABLED, INTERPOLATED, SHOOT}
    public HoodState hoodState = HoodState.INTERPOL_DISABLED;

    private double hoodTarget = Globals.hoodShootPos;

    public void setHoodTarget(double target) {
        hoodTarget = target;

        if (hoodState == HoodState.INTERPOLATED) {
            robot.hood.setPosition(hoodTarget);
        }
    }

    public void updateState(HoodState newState) {
        double target = 0;
        switch (newState) {
            case INTERPOL_DISABLED:
                target = robot.hood.getPosition();
                break;
            case INTERPOLATED:
                target = hoodTarget;
                break;
            case INIT:
                target = Globals.hoodInit;
                break;
            case SHOOT:
                target = Globals.hoodShootPos;
                break;
        }
        robot.hood.setPosition(target);
        hoodState = newState;
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
}
