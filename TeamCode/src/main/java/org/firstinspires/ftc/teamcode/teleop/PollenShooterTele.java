package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Pollen shooter TeleOp (FTCLib CommandOpMode + Pedro follower).
 *
 * LIVE TUNING (FTC Dashboard 192.168.43.1:8080/dash -> Config, then click SAVE,
 * or Panels 192.168.43.1:8001 -> Configurables):
 *   Globals -> shooterOnPollen    target velocity (ticks/s)   default 1650
 *   Globals -> kP/kI/kD/kF_Pollen hub velocity PIDF           default P 30, F 12.5
 *   Globals -> hoodShootPosPollen hood1 position              default 0.4
 *   PollenShooterTele -> FEED_DELAY_MS  wait between lock open and intake in
 *
 * Gamepad1:
 *   A (INIT only)     toggle field centric / robot centric
 *   BACK              toggle field centric / robot centric (during match)
 *   Left stick        drive (forward / strafe)
 *   Right stick X     turn
 *   Right stick btn   reset heading
 *   Y                 shooter on / off
 *   A                 feed toggle: feedLock1 PASS, then intake inward
 *                     (only while shooter is on; otherwise feedLock1 stays CLOSED)
 *   RIGHT BUMPER      hold: intake inward
 *   LEFT BUMPER       hold: intake outward
 */
@Config
@Configurable
@TeleOp(name = "Pollen Shooter Tele", group = "TeleOp")
public class PollenShooterTele extends CommandOpMode {

    public static double FEED_DELAY_MS = 200; // lock opens, then intake starts after this
    public static double TURN_SCALE = 0.7;

    private final RobotHardware robot = RobotHardware.getInstance();
    private Follower follower;
    private Telemetry tele; // Driver Station + Dashboard

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();

    public static boolean FIELD_CENTRIC = false; // A in init / BACK in match toggles it

    private boolean shooterOn = false;
    private boolean feeding = false;       // feed toggle (A)
    private boolean feedIntakeOn = false;  // set by the feed sequence after the delay

    private double lastP = Double.NaN, lastI = Double.NaN, lastD = Double.NaN, lastF = Double.NaN;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMs = 0, maxLoopMs = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);

        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        follower.update();

        applyPIDFIfChanged();

        while (opModeInInit()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            if (pressed(currentGamepad1.a, previousGamepad1.a)) {
                FIELD_CENTRIC = !FIELD_CENTRIC;
            }

            tele.addLine("Pollen Shooter Tele ready. Servos move after Start.");
            tele.addData("Drive (A to toggle)", FIELD_CENTRIC ? "FIELD CENTRIC" : "ROBOT CENTRIC");
            tele.addData("Target velocity", Globals.shooterOnPollen);
            tele.addData("PIDF", "P %.2f  I %.2f  D %.2f  F %.2f",
                    Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
            tele.addData("Hood1", Globals.hoodShootPosPollen);
            tele.update();
        }
        loopTimer.reset();
    }

    @Override
    public void run() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        maxLoopMs = Math.max(maxLoopMs, loopMs);

        robot.clearBulkCache();
        super.run(); // CommandScheduler.run()

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // ---------------- Drive ----------------
        if (pressed(currentGamepad1.back, previousGamepad1.back)) {
            FIELD_CENTRIC = !FIELD_CENTRIC;
        }

        double forward = -currentGamepad1.left_stick_y;
        double strafe = -currentGamepad1.left_stick_x;
        double turn = -currentGamepad1.right_stick_x * TURN_SCALE;

        if (FIELD_CENTRIC) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    forward, strafe, turn,
                    follower.pose().heading()
            );
            follower.manual(powers);
        } else {
            follower.manual(forward, strafe, turn);
        }
        follower.update();

        if (pressed(currentGamepad1.right_stick_button, previousGamepad1.right_stick_button)) {
            RobotHardware.resetYaw(follower);
        }

        // ---------------- Shooter on/off (Y) ----------------
        if (pressed(currentGamepad1.y, previousGamepad1.y)) {
            shooterOn = !shooterOn;
        }

        // ---------------- Feed toggle (A) ----------------
        if (pressed(currentGamepad1.a, previousGamepad1.a)) {
            if (feeding) {
                stopFeeding();
            } else if (shooterOn) {
                startFeeding();
            }
            // shooter off -> ignore, feedLock1 stays closed
        }
        // Shooter turned off while feeding -> stop and close the lock
        if (!shooterOn && feeding) {
            stopFeeding();
        }

        // ---------------- Live values (Dashboard / Panels) ----------------
        applyPIDFIfChanged();

        // ---------------- Shooter ----------------
        double target = shooterOn ? Math.max(0, Globals.shooterOnPollen) : 0;
        robot.shooter1.setVelocity(target);

        // ---------------- Servos ----------------
        robot.turret1.setPosition(Globals.turret1Init); // faces front
        robot.hood1.setPosition(Range.clip(Globals.hoodShootPosPollen,
                Math.min(Globals.hood1Up, Globals.hood1Down),
                Math.max(Globals.hood1Up, Globals.hood1Down)));
        robot.feedLock1.setPosition(feeding ? Globals.feedLock1Open : Globals.feedLock1Close);

        // ---------------- Intake ----------------
        double intakePower = 0;
        if (feedIntakeOn) {
            intakePower = Globals.intakeOn;
        } else if (gamepad1.right_bumper) {
            intakePower = Globals.intakeOn;
        } else if (gamepad1.left_bumper) {
            intakePower = Globals.intakeRev;
        }
        robot.intake1.setPower(intakePower);
        robot.intake2.setPower(intakePower);

        // ---------------- Telemetry ----------------
        double actual = robot.shooter1.getVelocity();
        tele.addData("Loop ms", "%.1f  (max %.1f)", loopMs, maxLoopMs);
        tele.addLine("Y shooter | A feed | RB in | LB out | RS btn reset heading");
        tele.addLine("------------------------------");
        tele.addData("Drive (BACK to toggle)", FIELD_CENTRIC ? "FIELD CENTRIC" : "ROBOT CENTRIC");
        tele.addData("Shooter", shooterOn ? "ON" : "OFF");
        tele.addData("Feeding", feeding ? (feedIntakeOn ? "LOCK OPEN + INTAKE" : "LOCK OPEN (waiting)") : "OFF (lock closed)");
        tele.addData("Intake power", intakePower);
        // Plain numbers so the Dashboard Graph tab can plot them.
        tele.addData("Target", target);
        tele.addData("Actual", actual);
        tele.addData("Error", target - actual);
        tele.addData("Shooter current (A)", "%.2f", robot.shooter1.getCurrent(CurrentUnit.AMPS));
        tele.addData("PIDF", "P %.2f  I %.2f  D %.2f  F %.2f",
                Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
        tele.addData("Hood1", Globals.hoodShootPosPollen);
        tele.addLine("------------------------------");
        tele.addData("x", "%.1f", follower.pose().x());
        tele.addData("y", "%.1f", follower.pose().y());
        tele.addData("heading", "%.1f", Math.toDegrees(follower.pose().heading()));
        tele.update();
    }

    /** feedLock1 PASS, wait FEED_DELAY_MS, then intake inward. */
    private void startFeeding() {
        feeding = true;
        feedIntakeOn = false;
        schedule(new SequentialCommandGroup(
                new WaitCommand((long) FEED_DELAY_MS),
                new InstantCommand(() -> {
                    if (feeding) feedIntakeOn = true; // skip if cancelled meanwhile
                })
        ));
    }

    /** Intake stops, feedLock1 CLOSED. */
    private void stopFeeding() {
        feeding = false;
        feedIntakeOn = false;
    }

    private static boolean pressed(boolean now, boolean last) {
        return now && !last;
    }

    /** Sends PIDF to the hub only when a value changed (Dashboard / Panels edit). */
    private void applyPIDFIfChanged() {
        double p = Math.max(0, Globals.kP_Pollen);
        double i = Math.max(0, Globals.kI_Pollen);
        double d = Math.max(0, Globals.kD_Pollen);
        double f = Math.max(0, Globals.kF_Pollen);
        if (p == lastP && i == lastI && d == lastD && f == lastF) return;
        robot.shooter1.setVelocityPIDFCoefficients(p, i, d, f);
        lastP = p;
        lastI = i;
        lastD = d;
        lastF = f;
    }
}
