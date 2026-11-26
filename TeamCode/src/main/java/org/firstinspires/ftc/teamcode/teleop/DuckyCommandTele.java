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

import org.firstinspires.ftc.teamcode.hardware.PrintData;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.turret1_ss;

/*
 * DuckyCommandTele - FTCLib command-based version of Scrimmage3Tele ("ducky")
 * with intake deploy (flower ramp) and turret1_ss odometry tracking.
 *
 * ======================= INIT =======================
 *   G1 LEFT BUMPER    toggle alliance RED / BLUE (carried over from auto by default)
 *
 * ======================= DRIVE (gamepad1) =======================
 *   FIELD CENTRIC from the DRIVER's view: stick forward = away from your
 *   drive-team wall. RED drivers stand at x = 0 (forward = +x, RED_DRIVER_DEG = 0),
 *   BLUE drivers at x = 141.5 (forward = -x, BLUE_DRIVER_DEG = 180).
 *   Left stick        drive (forward / strafe)
 *   Right stick X     turn
 *
 * ======================= INTAKE / SHOOT (gamepad1) =======================
 *   RIGHT BUMPER      intake toggle:
 *                       ON  -> intake deploy DOWN + intake inward
 *                       OFF -> intake stops + deploy UP
 *                     At 4 balls (front beam counter) the deploy goes UP,
 *                     intake stops / feeds the last ball, gamepad rumbles.
 *   X                 intake reverse toggle (spit out). Deploy stays where it is.
 *   LEFT BUMPER       SHOOT (needs ballCount > 0): shooter on, pollen lock open,
 *                     intake feeds, deploy UP. Ends FINISH_DELAY_MS after the
 *                     count reaches 0 (or after SHOOT_TIMEOUT_MS as a safety).
 *   BACK              STOP ALL: intake off, deploy UP, shooter off, lock closed,
 *                     shooter auto-start paused until next ball is counted.
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
 *   Gamepad2 is not used.
 *
 * Live values: Dashboard Config -> DuckyCommandTele / Panels Configurables.
 */
@Config
@Configurable
@TeleOp(name = "Ducky Command Tele", group = "TeleOp")
public class DuckyCommandTele extends CommandOpMode {

    // ---------- live-tunable ----------
    public static Pose startPose = new Pose(9, 9, 0);   // set by auto / default red bottom corner
    public static double RED_DRIVER_DEG = 0;
    public static double BLUE_DRIVER_DEG = 180;
    public static double TURN_SCALE = 1.0;
    public static int MAX_BALLS = 4;
    public static double FINISH_DELAY_MS = 250;   // keep feeding after count hits 0 (last ball fully out)
    public static double SHOOT_TIMEOUT_MS = 3000; // safety if a beam misses a ball
    public static int RUMBLE_MS = 300;

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
    private boolean intakeEnabled = false;
    private boolean intakeReversed = false;
    private boolean shooterEnabled = false;
    private boolean shooterPaused = false;
    private boolean isShooting = false;
    private boolean fullHandled = false;      // 4-ball actions done once per fill
    private Command finishCommand = null;     // delayed end of a shot sequence
    private final ElapsedTime shootTimer = new ElapsedTime();

    private boolean lastFrontBeam = true, lastPollenBeam = true;

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

        // Init positions are only sent at Start (see firstRun below)
        boolean lastLB = false;
        while (opModeInInit()) {
            if (gamepad1.left_bumper && !lastLB) RobotHardware.toggleAlliance();
            lastLB = gamepad1.left_bumper;

            telemetry.addLine("Initialized - G1 LB toggles alliance");
            telemetry.addData("Alliance", RobotHardware.getAlliance());
            telemetry.addData("Start pose", "(%.1f, %.1f, %.0f deg)",
                    startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
            telemetry.update();
        }
    }

    private boolean firstRun = true;

    @Override
    public void run() {
        robot.clearBulkCache();
        robot.update();
        robot.readGamepads(gamepad1, gamepad2); // for PrintData menu (gamepad1)

        if (firstRun) {
            onStart();
            firstRun = false;
        }

        super.run(); // CommandScheduler: buttons + scheduled commands

        drive();
        countBalls();
        handleFull();
        handleShooterAuto();
        handleShootEnd();

        turret1.update(follower.pose());

        PrintData.menu(robot.currentGamepad1, robot.previousGamepad1, telemetry);
        PrintData.print(robot, follower, intakeSS, shooterSS, null, turret1.tracking, shooterEnabled, ballCount, telemetry);
        telemetry.addData("Balls", ballCount);
        telemetry.addData("Intake", intakeReversed ? "REVERSE" : intakeEnabled ? "ON" : "OFF");
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
        intakeSS.updateState(IntakeSS.IntakeState.INIT);
        intakeSS.updateState(IntakeSS.FlowerRampState.INIT);
        turret1.center();
        robot.turret2.setPosition(0.5);

        lastFrontBeam = robot.frontBeam.getState();
        lastPollenBeam = robot.pollenBeam.getState();
        if (!lastFrontBeam) ballCount = 1; // a ball already sitting at the front beam
    }

    // ======================= BUTTONS (FTCLib) =======================
    private void bindButtons() {
        // ---- gamepad1 ----
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(this::toggleIntake));
        g1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(this::toggleReverse));
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(this::startShooting));
        g1.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(this::stopAll));

        // ---- turret / pose (gamepad1) ----
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
        intakeEnabled = !intakeEnabled || intakeReversed;
        intakeReversed = false;

        if (intakeEnabled && ballCount < MAX_BALLS) {
            fullHandled = false;
            intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED); // deploy DOWN
            intakeSS.updateState(IntakeSS.IntakeState.ON);
        } else {
            intakeEnabled = false;
            intakeSS.updateState(IntakeSS.IntakeState.INIT);
            intakeSS.updateState(IntakeSS.FlowerRampState.UP);       // deploy UP
        }
    }

    private void toggleReverse() {
        if (isShooting) return;
        intakeReversed = !intakeReversed;
        intakeEnabled = intakeReversed;
        intakeSS.updateState(intakeReversed ? IntakeSS.IntakeState.REVERSE : IntakeSS.IntakeState.INIT);
    }

    /** Front beam falling edge = ball in, pollen beam rising edge while shooting = ball out. */
    private void countBalls() {
        boolean frontBeam = robot.frontBeam.getState();   // false = blocked
        boolean pollenBeam = robot.pollenBeam.getState();

        if (!frontBeam && lastFrontBeam && !intakeReversed && ballCount < MAX_BALLS) {
            ballCount++;
            shooterPaused = false;
        }
        if (isShooting && pollenBeam && !lastPollenBeam && ballCount > 0) {
            ballCount--;
        }
        lastFrontBeam = frontBeam;
        lastPollenBeam = pollenBeam;
    }

    /** At MAX_BALLS: deploy UP (once), rumble, then stop intake or keep feeding the last ball. */
    private void handleFull() {
        if (ballCount < MAX_BALLS || !intakeEnabled || intakeReversed || isShooting) return;

        if (!fullHandled) {
            intakeSS.updateState(IntakeSS.FlowerRampState.UP);
            gamepad1.rumble(RUMBLE_MS);
            fullHandled = true;
        }
        // Pollen beam blocked -> balls packed, stop. Otherwise feed so the stack moves up.
        if (!robot.pollenBeam.getState()) {
            if (intakeSS.getIntakeState() != IntakeSS.IntakeState.INIT)
                intakeSS.updateState(IntakeSS.IntakeState.INIT);
        } else {
            if (intakeSS.getIntakeState() != IntakeSS.IntakeState.FEED)
                intakeSS.updateState(IntakeSS.IntakeState.FEED);
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

        intakeSS.updateState(IntakeSS.FlowerRampState.UP);
        shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED);
        intakeSS.updateState(IntakeSS.IntakeState.ON);
        intakeReversed = false;
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

    private void finishShooting() {
        finishCommand = null;
        shooterSS.updateState(ShooterSS.LockState.LOCKED);
        shooterOff();
        isShooting = false;

        if (intakeEnabled) {
            fullHandled = false;
            intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED);
            intakeSS.updateState(IntakeSS.IntakeState.ON);
        } else {
            intakeSS.updateState(IntakeSS.IntakeState.INIT);
        }
        lastFrontBeam = robot.frontBeam.getState();
    }

    private void stopAll() {
        if (finishCommand != null) {
            finishCommand.cancel();
            finishCommand = null;
        }
        intakeEnabled = false;
        intakeReversed = false;
        isShooting = false;
        shooterPaused = true;

        intakeSS.updateState(IntakeSS.IntakeState.INIT);
        intakeSS.updateState(IntakeSS.FlowerRampState.UP);
        shooterOff();
        shooterSS.updateState(ShooterSS.LockState.LOCKED);
    }
}
