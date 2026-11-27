package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class IntakeSS {
    RobotHardware robot = RobotHardware.getInstance();

    public IntakeSS(RobotHardware robot) {
        this.robot = robot;
    }

    public enum IntakeState {ON, INIT, REVERSE, FEED, POINT_FIVE}
    public IntakeState intakeState = IntakeState.INIT;

    public enum IntakeMode {ON, REVERSE, INIT}
    private IntakeMode currentMode = IntakeMode.INIT;

    //***edge detection***
    private boolean lastFrontBeam = false;
    private boolean lastPollenBeam = false;

    //TODO COUNT VARIABLE
    public enum Count {EMPTY, ONE, TWO, THREE, FOUR}
    private Count ballCount = Count.EMPTY;

    public void resetBallCount() {
        ballCount = Count.EMPTY;
        lastFrontBeam = robot.frontBeam.getState();
        lastPollenBeam = robot.pollenBeam.getState();
    }

    public void initBallCount() {
        resetBallCount();

        if (!lastFrontBeam) {
            ballCount = Count.ONE;
        }
    }

    public Count getBallCount() {
        return ballCount;
    }

    public int getBallCountValue() {
        return ballCount == Count.EMPTY ? 0 : ballCount == Count.ONE ? 1 : ballCount == Count.TWO ? 2 : ballCount == Count.THREE ? 3 : 4;
    }

    public void increaseCount() {
        switch (ballCount) {
            case EMPTY:
                ballCount = Count.ONE;
                break;
            case ONE:
                ballCount = Count.TWO;
                break;
            case TWO:
                ballCount = Count.THREE;
                break;
            case THREE:
                ballCount = Count.FOUR;
                break;
            case FOUR:
                break;
        }
    }

    public void decreaseCount() {
        switch (ballCount) {
            case FOUR:
                ballCount = Count.THREE;
                break;
            case THREE:
                ballCount = Count.TWO;
                break;
            case TWO:
                ballCount = Count.ONE;
                break;
            case ONE:
                ballCount = Count.EMPTY;
                break;
            case EMPTY:
                break;
        }
    }

    public boolean updateBallCount(boolean shooting) {
        Count previousCount = ballCount;
        boolean frontBeam = robot.frontBeam.getState();
        boolean pollenBeam = robot.pollenBeam.getState();

        //todo intake beambreaker logic
        if (!frontBeam && lastFrontBeam) {
            increaseCount();
        }

        /* if front beam not tripped, then lastFrontbeam = true, else if front beam tripped, then last front beam = false
         * so if front beam wasn't tripped and front beam is now tripped, ball count increases by one
         */

        if (shooting && pollenBeam && !lastPollenBeam) {
            decreaseCount();
        }

        lastFrontBeam = frontBeam;
        lastPollenBeam = pollenBeam;

        return ballCount == Count.FOUR && previousCount != Count.FOUR;
    }

    public IntakeMode getIntakeMode() {
        return currentMode;
    }

    void setIntakeMode(IntakeMode mode) {
        currentMode = mode;
    }

    public void stopIntake() {
        currentMode = IntakeMode.INIT;
        updateState(IntakeState.INIT);
    }

    public void toggleIntake(IntakeMode mode, ShooterSS.ShootingMode shootingMode) {
        if (shootingMode == ShooterSS.ShootingMode.RELOCK || (ballCount == Count.FOUR && shootingMode == ShooterSS.ShootingMode.INIT)) {
            return;
        }

        currentMode = currentMode == mode ? IntakeMode.INIT : mode;
    }

    public void updateIntake(ShooterSS.ShootingMode shootingMode) {
        IntakeState targetState;

        if (shootingMode == ShooterSS.ShootingMode.RELOCK || (ballCount == Count.FOUR && shootingMode == ShooterSS.ShootingMode.INIT)) {
            currentMode = IntakeMode.INIT;
            targetState = IntakeState.INIT;
        }
        else {
            switch (currentMode) {
                case ON:
                    targetState = shootingMode != ShooterSS.ShootingMode.INIT || ballCount == Count.EMPTY || ballCount == Count.ONE ? IntakeState.ON : IntakeState.POINT_FIVE;
                    break;
                case REVERSE:
                    targetState = IntakeState.REVERSE;
                    break;
                case INIT:
                default:
                    targetState = IntakeState.INIT;
                    break;
            }
        }

        if (intakeState != targetState) {
            updateState(targetState);
        }
    }

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

    public void toggleFlowerRamp() {
        updateState(flowerRampState == FlowerRampState.DEPLOYED ? FlowerRampState.UP : FlowerRampState.DEPLOYED);
    }

    public FlowerRampState getFlowerRampState() {
        return flowerRampState;
    }
}