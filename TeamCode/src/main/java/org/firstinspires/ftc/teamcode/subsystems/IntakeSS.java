package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class IntakeSS {
    RobotHardware robot = RobotHardware.getInstance();
    public IntakeSS (RobotHardware robot) {
        this.robot = robot;
    }
    public enum IntakeState {ON, INIT, REVERSE}
    public IntakeState intakeState = IntakeState.INIT;
    public void updateState(IntakeState newState) {
        double target = 0;
        switch (newState) {
            case ON:
                target = Globals.intakeOn;
                break;
            case INIT:
                target = Globals.intakeOff;
                break;
            case REVERSE:
                target = Globals.intakeRev;
                break;

        }
        robot.intake1.setPower(target);
        robot.intake2.setPower(target);
        intakeState = newState;
    }
    public IntakeState getState() {
        return intakeState;
    }
}