package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.PrintData;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.turret1_ss;

/*
 * DuckyTeleV2 - FTCLib command-based TeleOp. DuckyCommandTele + the best parts
 * of ParaTeleS3 (main): intake power steps down as the robot fills up.
 * ALL controls on gamepad1. Gamepad2 is not used.
 *
 * ======================= INIT (gamepad1) =======================
 *   LEFT BUMPER       toggle alliance RED / BLUE (carried over from auto by default)
 *   DPAD UP/DOWN      telemetry menu select, DPAD RIGHT toggle section
 *
 * ======================= DRIVE (gamepad1) =======================
 *   FIELD CENTRIC from the DRIVER's view: stick forward = away from your
 *   drive-team wall. RED drivers stand at x = 0 (forward = +x, RED_DRIVER_DEG = 0),
 *   BLUE drivers at x = 141.5 (forward = -x, BLUE_DRIVER_DEG = 180).
 *   Left stick        drive (forward / strafe)
 *   Right stick X     turn
 *
 * ======================= INTAKE / SHOOT (gamepad1) =======================
 *   RIGHT BUMPER      intake toggle ON (inward) / OFF. Does NOT move the deploy,
 *                     so you can intake from the field floor or from the flower.
 *   RIGHT TRIGGER     (> 0.5) intake deploy toggle DOWN <-> UP (for flower intake)
 *                     Intake power by ball count (front beam counter):
 *                       0 .. SLOW_FROM_BALLS-1  INTAKE_FULL_POWER
 *                       SLOW_FROM_BALLS .. 3    INTAKE_SLOW_POWER (less push on the lock)
 *                       4 (MAX_BALLS)           deploy UP (if it was down) + rumble,
 *                                               feed until the pollen beam sees the
 *                                               stack, then stop
 *   X                 intake reverse toggle (spit out). Balls are not counted
 *                     while reversing. Deploy stays where it is.
 *   LEFT BUMPER       SHOOT (needs ballCount > 0): shooter on, pollen lock open,
 *                     deploy UP (if not up), intake feeds at full power. Ends FINISH_DELAY_MS
 *                     after the count reaches 0 (or after SHOOT_TIMEOUT_MS as a safety).
 *   BACK              STOP ALL: intake off, deploy UP, shooter off, lock closed,
 *                     shooter auto-start paused until the next ball is counted.
 *
 * ======================= TURRET / POSE (gamepad1) =======================
 *   Y                 turret1 tracking on / off (off = turret centred)
 *   B                 turret1 goal: AUTO (by field half) -> FORCE_UP -> FORCE_DOWN
 *   RIGHT STICK BTN   reset pose: corner on YOUR RIGHT (seen from your drive wall)
 *                       RED (9, 9, 0)            BLUE (132.5, 132.5, 180)
 *   LEFT STICK BTN    reset pose: corner on YOUR LEFT
 *                       RED (9, 132.5, 0)        BLUE (132.5, 9, 180)
 *                     Push the robot into the corner facing into the field first.
 *
 * ======================= TELEMETRY MENU (gamepad1) =======================
 *   DPAD UP/DOWN      select section, DPAD RIGHT toggle section (PrintData)
 *
 *   A and START are free (START + A together is the Driver Station pairing combo).
 *
 * Live values: Dashboard Config -> DuckyTeleV2 / Panels Configurables.
 */
@Config
@Configurable
@TeleOp(name = "Ducky Tele V2", group = "TeleOp")
public class DuckyTeleV2 extends CommandOpMode {

    // ---------- live-tunable ----------
    public static Pose startPose = new Pose(9, 9, 0);   // set by auto / default red right corner
    public static double RED_DRIVER_DEG = 0;
    public static double BLUE_DRIVER_DEG = 180;
    public static double TURN_SCALE = 1.0;

    public static int MAX_BALLS = 4;
    public static double INTAKE_FULL_POWER = Globals.intakeOn; // 0 .. SLOW_FROM_BALLS-1 balls, and when shooting
    public static double INTAKE_SLOW_POWER = 0.5;              // from SLOW_FROM_BALLS balls
    public static int SLOW_FROM_BALLS = 2;

    public static double FINISH_DELAY_MS = 100;   // keep feeding after count hits 0 (last ball out)
    public static double SHOOT_TIMEOUT_MS = 3000; // safety if a beam misses a ball
    public static int RUMBLE_MS = 250;

    // Corner reset poses (x, y inches, heading deg), same as Turret1SSTest
    public static double RED_RS_X = 9,      RED_RS_Y = 9,      RED_RS_H = 0;    // RED  right stick btn
    public static double RED_LS_X = 9,      RED_LS_Y = 132.5,  RED_LS_H = 0;    // RED  left stick btn
    public static double BLUE_RS_X = 132.5, BLUE_RS_Y = 132.5, BLUE_RS_H = 180; // BLUE right stick btn
    public static double BLUE_LS_X = 132.5, BLUE_LS_Y = 9,     BLUE_LS_H = 180; // BLUE left stick btn

    public static int ballCount = 0;

    // ---------- hardware / subsystems ----------
    private final RobotHardware robot = RobotHardware.getInstance();
    private IntakeSS intakeSS;
    private ShooterSS shooterSS;
    private turret1_ss turret1;
    private Follower follower;
    private GamepadEx g1;

    // ---------- state ----------
    private enum IntakeMode { OFF, IN, REVERSE }
    private IntakeMode intakeMode = IntakeMode.OFF;
    private boolean shooterEnabled = false;
    private boolean shooterPaused = false;
    private boolean isShooting = false;
    private Command finishCommand = null;     // delayed end of a shot sequence
    private final ElapsedTime shootTimer = new ElapsedTime();

    private boolean lastFrontBeam = true, lastPollenBeam = true;
    private int lastBallCount = 0;
    private boolean firstRun = true;
    private boolean lastRT = false;

    // last powers sent to the intake motors (only send changes)
    private double sent1 = Double.NaN, sent2 = Double.NaN;
    private String intakeLabel = "OFF";

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap);

        intakeSS = new IntakeSS(robot);
        shooterSS = new ShooterSS(robot);
        turret1 = new turret1_ss(robot);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        ballCount = 0;

        g1 = new GamepadEx(gamepad1);
        bindButtons();

        // Servos / motors only move at Start (see onStart)
        while (opModeInInit()) {
            robot.readGamepads(gamepad1, gamepad2);
            if (robot.currentGamepad1.left_bumper && !robot.previousGamepad1.left_bumper) {
                RobotHardware.toggleAlliance();
            }

            telemetry.addLine("Initialized - G1 LB toggles alliance");
            telemetry.addData("Alliance", RobotHardware.getAlliance());
            telemetry.addData("Start pose", "(%.1f, %.1f, %.0f deg)",
                    startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
            PrintData.menu(robot.currentGamepad1, robot.previousGamepad1, telemetry);
            telemetry.update();
        }
    }

    @Override
    public void run() {
        robot.clearBulkCache();
        robot.update();
        robot.readGamepads(gamepad1, gamepad2); // for the PrintData menu (gamepad1)

        if (firstRun) {
            onStart();
            firstRun = false;
        }

        super.run(); // CommandScheduler: buttons + scheduled commands

        // Right trigger > 0.5: deploy toggle DOWN <-> UP (edge detected)
        boolean rt = gamepad1.right_trigger > 0.5;
        if (rt && !lastRT) toggleDeploy();
        lastRT = rt;

        drive();
        countBalls();
        updateIntake();
        handleShooterAuto();
        handleShootEnd();

        turret1.update(follower.pose());

        PrintData.menu(robot.currentGamepad1, robot.previousGamepad1, telemetry);
        PrintData.print(robot, follower, intakeSS, shooterSS, null, turret1.tracking, shooterEnabled, ballCount, telemetry);
        telemetry.addData("Balls", ballCount);
        telemetry.addData("Intake", intakeLabel);
        telemetry.addData("Deploy", intakeSS.getFlowerRampState());
        telemetry.addData("Shooting", isShooting);
        telemetry.addData("Turret", turret1.tracking ? ("TRACK " + turret1.activeHive + " (" + turret1.goalMode + ")") : "CENTRED");
        telemetry.update();
    }

    /** First loop after Start: move servos to their init positions. */
    private void onStart() {
        shooterSS.updateState(ShooterSS.ShooterState.INIT);
        shooterSS.updateState(ShooterSS.HoodState.SHOOT);
        shooterSS.updateState(ShooterSS.LockState.LOCKED);
        intakeSS.updateState(IntakeSS.FlowerRampState.INIT);
        setIntake(0, 0, "OFF");
        turret1.center();
        robot.turret2.setPosition(0.5);

        lastFrontBeam = robot.frontBeam.getState();
        lastPollenBeam = robot.pollenBeam.getState();
        if (!lastFrontBeam) ballCount = 1; // a ball already sitting at the front beam
        lastBallCount = ballCount;
    }

    // ======================= BUTTONS (FTCLib) =======================
    private void bindButtons() {
        // ---- intake / shoot ----
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(this::toggleIntake));
        g1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(this::toggleReverse));
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(this::startShooting));
        g1.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(this::stopAll));

        // ---- turret / pose ----
        g1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> schedule(turret1.trackCommand(!turret1.tracking))));
        g1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(turret1::cycleGoalMode));
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> cornerReset(true)));
        g1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> cornerReset(false)));
    }

    // ======================= DRIVE =======================
    private void drive() {
        double driverDeg = RobotHardware.isRed() ? RED_DRIVER_DEG : BLUE_DRIVER_DEG;
        follower.manual(ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * TURN_SCALE,
                follower.pose().heading() - Math.toRadians(driverDeg)
        ));
        follower.update();
    }

    /** Corner pose reset for the current alliance (right / left as seen from your drive wall). */
    private void cornerReset(boolean rightStick) {
        boolean red = RobotHardware.isRed();
        double x, y, h;
        if (red) {
            x = rightStick ? RED_RS_X : RED_LS_X;
            y = rightStick ? RED_RS_Y : RED_LS_Y;
            h = rightStick ? RED_RS_H : RED_LS_H;
        } else {
            x = rightStick ? BLUE_RS_X : BLUE_LS_X;
            y = rightStick ? BLUE_RS_Y : BLUE_LS_Y;
            h = rightStick ? BLUE_RS_H : BLUE_LS_H;
        }
        follower.setPose(new Pose(x, y, Math.toRadians(h)));
    }

    // ======================= INTAKE =======================
    private void toggleIntake() {
        if (isShooting) return;
        intakeMode = (intakeMode != IntakeMode.IN && ballCount < MAX_BALLS) ? IntakeMode.IN : IntakeMode.OFF;
    }

    /** Right trigger: flower deploy DOWN <-> UP. */
    private void toggleDeploy() {
        intakeSS.updateState(intakeSS.getFlowerRampState() == IntakeSS.FlowerRampState.DEPLOYED
                ? IntakeSS.FlowerRampState.UP
                : IntakeSS.FlowerRampState.DEPLOYED);
    }

    private void toggleReverse() {
        if (isShooting) return;
        intakeMode = intakeMode == IntakeMode.REVERSE ? IntakeMode.OFF : IntakeMode.REVERSE;
    }

    /** Front beam falling edge = ball in, pollen beam rising edge while shooting = ball out. */
    private void countBalls() {
        boolean frontBeam = robot.frontBeam.getState();   // false = blocked
        boolean pollenBeam = robot.pollenBeam.getState();

        if (!frontBeam && lastFrontBeam && intakeMode != IntakeMode.REVERSE && ballCount < MAX_BALLS) {
            ballCount++;
            shooterPaused = false;
        }
        if (isShooting && pollenBeam && !lastPollenBeam && ballCount > 0) {
            ballCount--;
        }
        lastFrontBeam = frontBeam;
        lastPollenBeam = pollenBeam;

        // Just became full: deploy UP + rumble (once per fill)
        if (ballCount >= MAX_BALLS && lastBallCount < MAX_BALLS && intakeMode == IntakeMode.IN && !isShooting) {
            if (intakeSS.getFlowerRampState() == IntakeSS.FlowerRampState.DEPLOYED) {
                intakeSS.updateState(IntakeSS.FlowerRampState.UP);
            }
            gamepad1.rumble(RUMBLE_MS);
        }
        lastBallCount = ballCount;
    }

    /** Works out the intake powers every loop from the current state; only sends changes. */
    private void updateIntake() {
        if (isShooting) {
            setIntake(INTAKE_FULL_POWER, INTAKE_FULL_POWER, "SHOOT FEED");
            return;
        }
        switch (intakeMode) {
            case REVERSE:
                setIntake(Globals.intakeRev, Globals.intakeRev, "REVERSE");
                break;
            case IN:
                if (ballCount >= MAX_BALLS) {
                    // Full: push the stack up until the pollen beam sees it, then stop
                    if (robot.pollenBeam.getState()) setIntake(0, INTAKE_SLOW_POWER, "FULL - FEED UP");
                    else setIntake(0, 0, "FULL - STOP");
                } else if (ballCount >= SLOW_FROM_BALLS) {
                    setIntake(INTAKE_SLOW_POWER, INTAKE_SLOW_POWER, "IN SLOW");
                } else {
                    setIntake(INTAKE_FULL_POWER, INTAKE_FULL_POWER, "IN FULL");
                }
                break;
            default:
                setIntake(0, 0, "OFF");
                break;
        }
    }

    private void setIntake(double p1, double p2, String label) {
        intakeLabel = label;
        if (p1 != sent1) {
            robot.intake1.setPower(p1);
            sent1 = p1;
        }
        if (p2 != sent2) {
            robot.intake2.setPower(p2);
            sent2 = p2;
        }
    }

    // ======================= SHOOTER =======================
    private void shooterOn() {
        shooterEnabled = true;
        shooterSS.updatePollenState(ShooterSS.ShooterState.INTERPOL_DISABLED); // Globals.shooterOnPollen
    }

    private void shooterOff() {
        shooterEnabled = false;
        shooterSS.updatePollenState(ShooterSS.ShooterState.INIT);
    }

    /** Spin up as soon as there is a ball; spin down when empty. */
    private void handleShooterAuto() {
        if (ballCount >= 1 && !shooterEnabled && !shooterPaused) shooterOn();
        else if (ballCount == 0 && shooterEnabled && !isShooting) shooterOff();
    }

    private void startShooting() {
        if (ballCount <= 0 || isShooting) return;
        shooterPaused = false;
        if (!shooterEnabled) shooterOn();

        deployUpIfNot();
        shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED);
        if (intakeMode == IntakeMode.REVERSE) intakeMode = IntakeMode.OFF;
        isShooting = true;
        shootTimer.reset();
    }

    /** Count hits 0 -> finish after FINISH_DELAY_MS (cancelled if a ball is counted again). */
    private void handleShootEnd() {
        if (!isShooting) return;

        if (shootTimer.milliseconds() > SHOOT_TIMEOUT_MS) {
            ballCount = 0; // a beam missed a ball; don't stay stuck shooting
        }

        if (ballCount == 0) {
            if (finishCommand == null) {
                finishCommand = new SequentialCommandGroup(
                        new WaitCommand((long) FINISH_DELAY_MS),
                        new InstantCommand(this::finishShooting)
                );
                schedule(finishCommand);
            }
        } else if (finishCommand != null) {
            finishCommand.cancel();
            finishCommand = null;
        }
    }

    /** Deploy UP only if it is not already up. */
    private void deployUpIfNot() {
        if (intakeSS.getFlowerRampState() != IntakeSS.FlowerRampState.UP) {
            intakeSS.updateState(IntakeSS.FlowerRampState.UP);
        }
    }

    private void finishShooting() {
        finishCommand = null;
        shooterSS.updateState(ShooterSS.LockState.LOCKED);
        shooterOff();
        isShooting = false;

        deployUpIfNot(); // stays UP after shooting; RT puts it down again when needed
        // Intake goes back to its previous mode automatically (updateIntake)
        lastFrontBeam = robot.frontBeam.getState();
        lastBallCount = ballCount;
    }

    private void stopAll() {
        if (finishCommand != null) {
            finishCommand.cancel();
            finishCommand = null;
        }
        intakeMode = IntakeMode.OFF;
        isShooting = false;
        shooterPaused = true;

        intakeSS.updateState(IntakeSS.FlowerRampState.UP);
        shooterOff();
        shooterSS.updateState(ShooterSS.LockState.LOCKED);
    }
}
