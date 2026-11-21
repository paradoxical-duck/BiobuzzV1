
package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.PrintData;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSS;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSS;

@TeleOp(name = "Para Tele V3")
public class STeleOpV3 extends LinearOpMode {
    IntakeSS intakeSS;
    ShooterSS shooterSS;
    TrackingSS trackingSS;
    ColorSS colorSS;
    Follower follower;
    RobotHardware robot = RobotHardware.getInstance();
    public static Pose startPose = new Pose(0, 0, 0);
    boolean red = false;
    boolean trackingEnabled = false;
    boolean shooterEnabled = false;
    boolean intakeEnabled = false;
    boolean intakeReversed = false;

    //***edge detection***
    boolean lastFrontBeam = false;

    //TODO COUNT VARIABLE
    public static int ballCount;

    //TODO BALL COLORS

    boolean isShooting = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //***subsystems***
        intakeSS = new IntakeSS(robot);
        shooterSS = new ShooterSS(robot);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        shooterSS.updateState(ShooterSS.ShooterState.INIT);
        shooterSS.updateState(ShooterSS.HoodState.INIT);
        shooterSS.updateState(ShooterSS.TurretState.INIT);
        intakeSS.updateState(IntakeSS.IntakeState.INIT);

        ballCount = 0;
        robot.applyShooterPIDF();
        while (opModeInInit()) {
            red = RobotHardware.isRed();
            //alliance selection carried over from auto but can still be manually changed
            if (gamepad1.left_bumper && !robot.previousGamepad1.left_bumper) {
                red = !red;
            }

            //since red = RobotHardware.isRed() it is carried over from auto by default, however can be manually changed
            if (red) {
                RobotHardware.setAlliance(RobotHardware.Alliance.RED);
            }
            else {
                RobotHardware.setAlliance(RobotHardware.Alliance.BLUE);
            }
            telemetry.addLine("Initialized");
            telemetry.addLine(RobotHardware.getAlliance() + " alliance active");
            PrintData.menu(gamepad1, robot.previousGamepad1, telemetry);
            telemetry.update();
            robot.previousGamepad1.copy(gamepad1);
        }

        trackingSS = new TrackingSS(robot, follower, RobotHardware.getAlliance());
        colorSS = new ColorSS(robot, RobotHardware.getAlliance());
        colorSS.reset();

        waitForStart();
        robot.clearBulkCache();

        lastFrontBeam = robot.frontBeam.getState();

        if (!lastFrontBeam) {
            ballCount = 1;
        }

        while (opModeIsActive()) {
            robot.clearBulkCache();
            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            follower.update();
            colorSS.update(ballCount);

            //todo tracking logic
            if (gamepad1.start && !robot.previousGamepad1.start) {
                trackingEnabled = !trackingEnabled;

                if (trackingEnabled) {
                    TrackingSS.updateTracking(shooterSS);
                    shooterSS.updateState(ShooterSS.TurretState.TRACKING_ENABLED);
                    shooterSS.updateState(ShooterSS.HoodState.INTERPOLATED);
                }
                else {
                    shooterSS.updateState(ShooterSS.TurretState.TRACKING_DISABLED);
                    shooterSS.updateState(ShooterSS.HoodState.SHOOT);
                }

                if (shooterEnabled && trackingEnabled) {
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOLATED);
                }
                else if (shooterEnabled) {
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOL_DISABLED);
                }
            }

            //todo intake logic
            if (gamepad1.right_bumper && !robot.previousGamepad1.right_bumper) {
                intakeEnabled = !intakeEnabled;
                intakeReversed = false;

                if (intakeEnabled && ballCount < 4) {
                    intakeSS.updateState(IntakeSS.IntakeState.ON);
                }
                else {
                    intakeEnabled = false;
                    intakeSS.updateState(IntakeSS.IntakeState.INIT);
                }
            }

            if (gamepad1.x && !robot.previousGamepad1.x) {
                intakeReversed = !intakeReversed;
                intakeEnabled = intakeReversed;

                if (intakeReversed) {
                    intakeSS.updateState(IntakeSS.IntakeState.REVERSE);
                }
                else {
                    intakeSS.updateState(IntakeSS.IntakeState.INIT);
                }
            }

            //todo intake beambreaker logic
            boolean frontBeam = robot.frontBeam.getState();

            if (!frontBeam && lastFrontBeam && ballCount < 4) {
                ballCount++;
            }
            lastFrontBeam = frontBeam;

            /* if front beam not tripped, then lastFrontbeam = true, else if front beam tripped, then last front beam = false
             * so if front beam wasn't tripped and front beam is now tripped, ball count increases by one
             */

            if (ballCount == 4 && intakeEnabled && !intakeReversed) {
                if (!robot.pollenBeam.getState() && !robot.nectarBeam.getState()) {
                    if (intakeSS.getIntakeState() != IntakeSS.IntakeState.INIT) {
                        intakeSS.updateState(IntakeSS.IntakeState.INIT);
                    }
                }
                else if (robot.pollenBeam.getState() && robot.nectarBeam.getState()) {
                    /*todo
                     * this is when 4 balls are there and the balls havent reached pollen and nectar shooters yet
                     * so, feeder stays on but intake is turned off so balls reach the shooters, but more balls don't come through the front intake
                     */
                    if (intakeSS.getIntakeState() != IntakeSS.IntakeState.FEED) {
                        intakeSS.updateState(IntakeSS.IntakeState.FEED);
                    }
                }
                else if (!robot.pollenBeam.getState() && robot.nectarBeam.getState() && colorSS.allPollen()) {
                    //since all pollen, all 4 balls are in correction positions
                    if (intakeSS.getIntakeState() != IntakeSS.IntakeState.INIT) {
                        intakeSS.updateState(IntakeSS.IntakeState.INIT);
                    }
                }
                else if (!robot.nectarBeam.getState() && !robot.midBeam.getState() && robot.pollenBeam.getState() && colorSS.allNectar()) {
                    //all are nectar, there is nectar in shooter and there is nectar in mid so all balls correct, intake off
                    if (intakeSS.getIntakeState() != IntakeSS.IntakeState.INIT) {
                        intakeSS.updateState(IntakeSS.IntakeState.INIT);
                    }
                }
                else if (!colorSS.allNectar() && !colorSS.allPollen() && (robot.nectarBeam.getState() || robot.pollenBeam.getState())) {
                    if (intakeSS.getIntakeState() != IntakeSS.IntakeState.FEED) {
                        intakeSS.updateState(IntakeSS.IntakeState.FEED);
                    }
                }
            }

            if (ballCount >= 1 && !shooterEnabled) {
                shooterEnabled = true;

                if (trackingEnabled) {
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOLATED);
                }
                else {
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOL_DISABLED);
                }
            }
            else if (ballCount == 0 && shooterEnabled) {
                shooterEnabled = false;
                shooterSS.updateState(ShooterSS.ShooterState.INIT);
            }

            if (gamepad1.a && !robot.previousGamepad1.a) {
                if (colorSS.allNectar()) {
                    shooterSS.updateState(ShooterSS.LockState.NECTAR_UNLOCKED);
                }
                else if (colorSS.allPollen()) {
                    shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED);
                }
                else {
                    shooterSS.updateState(ShooterSS.LockState.UNLOCKED);
                }
                isShooting = true;
            }

            if (isShooting && robot.isEmpty()) {
                shooterSS.updateState(ShooterSS.LockState.LOCKED);
                isShooting = false;
                ballCount = 0;
                colorSS.reset();
                lastFrontBeam = robot.frontBeam.getState();
            }

            if (trackingEnabled) {
                TrackingSS.updateTracking(shooterSS);
            }

            PrintData.print(robot, follower, intakeSS, shooterSS, colorSS, trackingEnabled, shooterEnabled, ballCount, telemetry);
            telemetry.update();
            robot.previousGamepad1.copy(gamepad1);
        }
    }
}
