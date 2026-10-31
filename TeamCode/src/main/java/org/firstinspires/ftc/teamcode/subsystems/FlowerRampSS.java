package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class FlowerRampSS {
    RobotHardware robot = RobotHardware.getInstance();
    public FlowerRampSS(RobotHardware robot) {
        this.robot = robot;
    }
    public enum FlowerRampState {UP, INIT, DEPLOYED /*todo - deployed state is intake, init and up state most likely same*/}
    public FlowerRampState flowerRampState = FlowerRampState.INIT;
    public void updateState(FlowerRampState newState) {
        double target = 0;
        switch (newState) {
            case UP:
                target = Globals.flowerRampUp;
                break;
            case DEPLOYED:
                target = Globals.flowerRampDeployed;
                break;
            case INIT:
                target = Globals.flowerRampInit;
                break;
        }
        robot.flowerRamp.setPosition(target);
        flowerRampState = newState;
    }
    public FlowerRampState getState() {
        return flowerRampState;
    }


}