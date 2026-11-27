package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class IntakeSS {
    RobotHardware robot = RobotHardware.getInstance();
    public IntakeSS (RobotHardware robot) {
        this.robot = robot;
    }
    public enum IntakeState {ON, INIT, REVERSE, FEED, POINT_FIVE}
    public IntakeState intakeState = IntakeState.INIT;
    public enum IntakeMode {ON, REVERSE, INIT}
    public void updateState(IntakeState newState) {
        double target1 = 0;
        double target2 = 0;
        switch (newState) {
            case ON:
                target1 = Globals.intakeOn;
                target2 = Globals.intakeOn;
                break;
            case INIT:
                target1 = Globals.intakeOff;
                target2 = Globals.intakeOff;
                break;
            case REVERSE:
                target1 = Globals.intakeRev;
                target2 = Globals.intakeRev;
                break;
            case FEED:
                target1 = Globals.intakeOff;
                target2 = Globals.intakeOn;
                break;
            case POINT_FIVE:
                target1 = 0.5;
                target2 = 0.5;
                break;
        }
        robot.intake1.setPower(target1);
        robot.intake2.setPower(target2);
        intakeState = newState;
    }
    public IntakeState getIntakeState() {
        return intakeState;
    }
    public enum FlowerRampState {UP, INIT, DEPLOYED /*todo - deployed state is intake, init and up state most likely same*/}
    public FlowerRampState flowerRampState = FlowerRampState.INIT;
    public void updateState(FlowerRampState newState) {
        double target = 0;
        switch (newState) {
            case UP:
                target = Globals.intakeDeployUp;
                break;
            case DEPLOYED:
                target = Globals.intakeDeployDown;
                break;
            case INIT:
                target = Globals.intakeDeployInit;
                break;
        }
        robot.intakeDeploy.setPosition(target);
        flowerRampState = newState;
    }
    public FlowerRampState getFlowerRampState() {
        return flowerRampState;
    }
}