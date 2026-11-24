package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

import java.util.List;

/*
 * Pollen shooter tuner (FTCLib CommandOpMode + FTC Dashboard).
 *
 * 1. Tune servos: turret1, hood1 (both start at 0.5) and feedLock1.
 *    Step them to each mechanical limit and note the values.
 * 2. Tune shooter1 velocity PIDF (REV hub's built-in velocity PIDF).
 *    Use the intake to feed balls while checking the shot.
 * Copy the final values into Globals yourself.
 *
 * LIVE TUNING: open http://192.168.43.1:8080/dash
 *   Config tab -> ShooterPollenTuner (servos, VELOCITY, SHOOTER_ON)
 *   Config tab -> Globals (kP_Pollen, kI_Pollen, kD_Pollen, kF_Pollen)
 *   !! After typing a value you MUST click the SAVE button (top-right of the
 *      Config card). Until then the value is only in your browser, not the robot.
 *   Gamepad changes are pushed back to the dashboard automatically.
 *   Backup: Panels at http://192.168.43.1:8001 -> Configurables (applies on edit).
 *   Graph tab  -> plots Target / Actual / Error velocity.
 *   The gamepad and the dashboard change the same values.
 *
 * Gamepad1:
 *   DPAD UP/DOWN      select parameter
 *   DPAD RIGHT/LEFT   increase / decrease selected value
 *   RB / LB           step x10 / step /10 (selected parameter)
 *   B                 selected servo back to 0.5
 *   Y                 shooter on / off (at target velocity)
 *   X                 stop shooter and intake
 *   A                 feedLock1 toggle CLOSE (block) <-> OPEN (pass)
 *   RIGHT TRIGGER     hold: intake inward
 *   LEFT TRIGGER      hold: intake outward
 */
@Config
@Configurable
@TeleOp(name = "Shooter Pollen Tuner", group = "Version Testing")
public class























ShooterPollenTuner extends CommandOpMode {

    // ===== Live-tunable values (FTC Dashboard Config tab) =====
    public static double TURRET1 = Globals.turret1Init;
    public static double HOOD1 = 0.5;
    public static double FEEDLOCK1 = Globals.feedLock1Close;
    public static double VELOCITY = Globals.shooterOnPollen; // ticks/s
    // P, I, D, F live in Globals (kP_Pollen ... kF_Pollen) -> tune them in Dashboard Config -> Globals
    public static boolean SHOOTER_ON = false;

    // Static so tuned PIDF/velocity survive an OpMode restart (until the app restarts).
    private static boolean pidfLoaded = false;

    private enum Param {
        TURRET1("Turret1", 0.01),
        HOOD1("Hood1", 0.01),
        FEEDLOCK1("FeedLock1", 0.01),
        VELOCITY("Target velocity (ticks/s)", 50),
        P("P", 1),
        I("I", 0.1),
        D("D", 0.1),
        F("F", 1);

        final String label;
        double step;

        Param(String label, double step) {
            this.label = label;
            this.step = step;
        }
    }

    private final Param[] params = Param.values();
    private int selected = 0;
    private String intakeState = "OFF";
    private boolean pushConfig = false;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private int loops = 0;
    private double lastP = Double.NaN, lastI = Double.NaN, lastD = Double.NaN, lastF = Double.NaN;

    private List<LynxModule> hubs;
    private DcMotorEx shooter1, intake1, intake2;
    private Servo turret1, hood1, feedLock1;
    private GamepadEx gp;
    private Telemetry tele; // Driver Station + Dashboard

    @Override
    public void initialize() {
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret1 = hardwareMap.get(Servo.class, "turret1");
        hood1 = hardwareMap.get(Servo.class, "hood1");
        feedLock1 = hardwareMap.get(Servo.class, "feedLock1");

        // Servos always start at 0.5 so you find limits from the middle.
        TURRET1 = Globals.turret1Init; // faces front
        HOOD1 = 0.5;
        FEEDLOCK1 = Globals.feedLock1Close; // starts blocked
        SHOOTER_ON = false;

        // First run only: if Globals PIDF is all 0, start from what the hub is using now.
        if (!pidfLoaded) {
            if (Globals.kP_Pollen == 0 && Globals.kI_Pollen == 0 && Globals.kD_Pollen == 0 && Globals.kF_Pollen == 0) {
                PIDFCoefficients hub = shooter1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                Globals.kP_Pollen = hub.p;
                Globals.kI_Pollen = hub.i;
                Globals.kD_Pollen = hub.d;
                Globals.kF_Pollen = hub.f;
            }
            pidfLoaded = true;
        }
        applyPIDFIfChanged();

        gp = new GamepadEx(gamepad1);
        gp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> selected = (selected + params.length - 1) % params.length));
        gp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> selected = (selected + 1) % params.length));
        gp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(cmd(() -> change(+1)));
        gp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(cmd(() -> change(-1)));
        gp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> params[selected].step *= 10));
        gp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> params[selected].step /= 10));
        gp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(cmd(() -> {
                    if (isServo(params[selected])) set(params[selected], 0.5);
                }));
        gp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(cmd(() -> SHOOTER_ON = !SHOOTER_ON));
        gp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(cmd(() -> SHOOTER_ON = false));
        // A: feedLock1 toggle CLOSE (block) <-> OPEN (pass), using Globals values
        gp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(cmd(() ->
                        FEEDLOCK1 = isLockOpen() ? Globals.feedLock1Close : Globals.feedLock1Open));

        // Show the init values on the dashboard (otherwise it shows old ones).
        FtcDashboard.getInstance().updateConfig();

        while (opModeInInit()) {
            tele.addLine("Shooter Pollen Tuner ready. Servos move after Start.");
            tele.addLine("Dashboard: 192.168.43.1:8080/dash  (edit value -> click SAVE)");
            tele.addData("Starting PIDF", "P %.3f  I %.3f  D %.3f  F %.3f", Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
            tele.update();
        }
    }

    @Override
    public void run() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        super.run(); // CommandScheduler.run(): handles the button bindings

        // Dashboard edits skip change(), so keep values in range and apply PIDF here.
        TURRET1 = Range.clip(TURRET1, 0, 1);
        HOOD1 = Range.clip(HOOD1, 0, 1);
        FEEDLOCK1 = Range.clip(FEEDLOCK1, 0, 1);
        applyPIDFIfChanged();

        // A gamepad button changed a value -> show it on the dashboard too.
        if (pushConfig) {
            FtcDashboard.getInstance().updateConfig();
            pushConfig = false;
        }

        // Intake: hold triggers. X stops everything.
        double in = gp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double out = gp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double intakePower = 0;
        if (gamepad1.x) {
            intakePower = 0;
        } else if (in > 0.3) {
            intakePower = Globals.intakeOn;
        } else if (out > 0.3) {
            intakePower = Globals.intakeRev;
        }
        intakeState = intakePower > 0 ? "INWARD" : intakePower < 0 ? "OUTWARD" : "OFF";
        intake1.setPower(intakePower);
        intake2.setPower(intakePower);

        turret1.setPosition(TURRET1);
        hood1.setPosition(HOOD1);
        feedLock1.setPosition(FEEDLOCK1);

        double target = SHOOTER_ON ? Math.max(0, VELOCITY) : 0;
        shooter1.setVelocity(target);

        showTelemetry(target);
    }

    /** Gamepad command that also tells the dashboard about the new value. */
    private InstantCommand cmd(Runnable r) {
        return new InstantCommand(() -> {
            r.run();
            pushConfig = true;
        });
    }

    private double get(Param p) {
        switch (p) {
            case TURRET1: return TURRET1;
            case HOOD1: return HOOD1;
            case FEEDLOCK1: return FEEDLOCK1;
            case VELOCITY: return VELOCITY;
            case P: return Globals.kP_Pollen;
            case I: return Globals.kI_Pollen;
            case D: return Globals.kD_Pollen;
            case F: return Globals.kF_Pollen;
        }
        return 0;
    }

    private void set(Param p, double v) {
        switch (p) {
            case TURRET1: TURRET1 = v; break;
            case HOOD1: HOOD1 = v; break;
            case FEEDLOCK1: FEEDLOCK1 = v; break;
            case VELOCITY: VELOCITY = v; break;
            case P: Globals.kP_Pollen = v; break;
            case I: Globals.kI_Pollen = v; break;
            case D: Globals.kD_Pollen = v; break;
            case F: Globals.kF_Pollen = v; break;
        }
    }

    private void change(int direction) {
        Param p = params[selected];
        double v = get(p) + direction * p.step;
        set(p, isServo(p) ? Range.clip(v, 0, 1) : Math.max(0, v));
    }

    /** Open if closer to the open value than the closed one (works after manual nudges too). */
    private boolean isLockOpen() {
        return Math.abs(FEEDLOCK1 - Globals.feedLock1Open) < Math.abs(FEEDLOCK1 - Globals.feedLock1Close);
    }

    private boolean isServo(Param p) {
        return p == Param.TURRET1 || p == Param.HOOD1 || p == Param.FEEDLOCK1;
    }

    /** Sends PIDF to the hub only when a value changed (gamepad or dashboard). */
    private void applyPIDFIfChanged() {
        Globals.kP_Pollen = Math.max(0, Globals.kP_Pollen);
        Globals.kI_Pollen = Math.max(0, Globals.kI_Pollen);
        Globals.kD_Pollen = Math.max(0, Globals.kD_Pollen);
        Globals.kF_Pollen = Math.max(0, Globals.kF_Pollen);
        if (Globals.kP_Pollen == lastP && Globals.kI_Pollen == lastI && Globals.kD_Pollen == lastD && Globals.kF_Pollen == lastF) return;
        shooter1.setVelocityPIDFCoefficients(Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
        lastP = Globals.kP_Pollen;
        lastI = Globals.kI_Pollen;
        lastD = Globals.kD_Pollen;
        lastF = Globals.kF_Pollen;
    }

    private void showTelemetry(double target) {
        double actual = shooter1.getVelocity();

        tele.addLine("DPAD U/D select | DPAD R/L +/- | RB/LB step x10 ÷10");
        tele.addLine("B servo→0.5 | Y shooter on/off | X stop | A lock toggle | RT in | LT out");
        tele.addLine();

        for (int i = 0; i < params.length; i++) {
            Param p = params[i];
            String fmt = isServo(p) ? "%.4f   (step %.4f)" : "%.3f   (step %.3f)";
            tele.addData((i == selected ? ">> " : "   ") + p.label, fmt, get(p), p.step);
        }

        tele.addLine();
        // Loops should keep counting; if not, the OpMode is stuck.
        tele.addData("Loop", "%d   (%.1f ms)", ++loops, loopTimer.milliseconds());
        loopTimer.reset();
        tele.addLine("******** SHOOTER1 ********");
        tele.addData("Shooter", SHOOTER_ON ? "ON" : "OFF");
        tele.addData("FeedLock1 (A)", isLockOpen() ? "OPEN (pass)" : "CLOSED (block)");
        // Plain numbers so the Dashboard Graph tab can plot them.
        tele.addData("Target", target);
        tele.addData("Actual", actual);
        tele.addData("Error", target - actual);
        tele.addData("Motor power", "%.2f", shooter1.getPower());
        tele.addData("Current (A)", "%.2f", shooter1.getCurrent(CurrentUnit.AMPS));
        tele.addData("Intake", intakeState);
        tele.addLine("Tip: tune F first (F ≈ 32767 / max ticks/s), then P.");
        tele.update();
    }
}
