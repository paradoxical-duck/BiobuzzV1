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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.VelocityFFController;

import java.util.List;

/*
 * Pollen shooter tuner 2 (FTCLib CommandOpMode + FTC Dashboard) - adds the
 * kS + kV + kP + kD VelocityFFController with voltage compensation and BOOST.
 *
 * 1. Tune servos: turret1, hood1 (both start at 0.5) and feedLock1.
 *    Step them to each mechanical limit and note the values.
 * 2. Tune shooter1 velocity. Two controllers, switch live with USE_CUSTOM:
 *    USE_CUSTOM = false  REV hub built-in PIDF (Globals kP/kI/kD/kF_Pollen)
 *    USE_CUSTOM = true   kS + kV + kP + kD, voltage compensated, with BOOST
 *    Use the intake to feed balls while checking the shot.
 * Copy the final values into Globals yourself.
 *
 * CUSTOM CONTROLLER TUNING ORDER (Dashboard Graph: Target, Actual, PowerX1000):
 *   0. BOOST_ENABLED = false, kP = 0, kD = 0.
 *   1. kS: kV = 0, shooter ON, raise kS until the wheel JUST starts to turn,
 *      then back off a little.
 *   2. kV: set kV until Actual sits on Target with kP = 0. Telemetry
 *      "kV estimate" shows the value that would hold the current speed.
 *      Check at 2-3 different VELOCITY values; it should stay the same.
 *   3. kP: raise until Actual snaps to Target after a change, stop before it
 *      oscillates. Typical 0.001 - 0.01.
 *   4. kD: only if it overshoots / rings. Small (0.00001 - 0.0001).
 *   5. BOOST_ENABLED = true. Feed balls (A lock open + RT intake).
 *      Watch "Last drop" and "Last recovery ms". Lower BOOST_ENTER for
 *      faster reaction, raise BOOST_EXIT if it overshoots after boosting.
 *   Battery: VOLTAGE_COMP keeps the same behaviour from 13.5 V to 12 V.
 *
 * LIVE TUNING: open http://192.168.43.1:8080/dash
 *   Config tab -> ShooterPollenTuner2 (servos, VELOCITY, SHOOTER_ON)
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
@TeleOp(name = "Shooter Pollen Tuner 2", group = "Version Testing")
public class ShooterPollenTuner2 extends CommandOpMode {

    // ===== Live-tunable values (FTC Dashboard Config tab) =====
    public static double TURRET1 = Globals.turret1Init;
    public static double HOOD1 = 0.5;
    public static double FEEDLOCK1 = Globals.feedLock1Close;
    public static double VELOCITY = Globals.shooterOnPollen; // ticks/s
    // P, I, D, F live in Globals (kP_Pollen ... kF_Pollen) -> tune them in Dashboard Config -> Globals
    public static boolean SHOOTER_ON = false;

    // ===== Custom velocity controller (kS + kV + kP + kD) =====
    public static boolean USE_CUSTOM = true;   // false = REV hub PIDF
    public static double kS = 0.05;            // power to overcome friction
    public static double kV = 0.00036;         // power per ticks/s (~1 / max ticks/s)
    public static double kP = 0.002;           // power per ticks/s of error
    public static double kD = 0.0;             // damping, power per ticks/s^2
    public static double D_FILTER = 0.3;       // 0..1 derivative low-pass (1 = raw)
    public static boolean VOLTAGE_COMP = true;
    public static double NOMINAL_VOLTAGE = 12.5;
    public static double VOLTAGE_FILTER = 0.1; // 0..1 low-pass on battery voltage

    // Boost: full power when far below target (after a shot), until close again
    public static boolean BOOST_ENABLED = false;
    public static double BOOST_ENTER = 100;    // ticks/s below target -> boost on
    public static double BOOST_EXIT = 30;      // ticks/s below target -> boost off
    public static double BOOST_POWER = 1.0;

    // Shot statistics
    public static double READY_TOL = 40;       // |error| under this = at speed
    public static double DROP_DETECT = 60;     // error over this (after ready) = shot

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

    private final VelocityFFController ff = new VelocityFFController();
    private VoltageSensor voltageSensor;
    private double batteryVoltage = 12.5;
    private Boolean lastUseCustom = null;

    // Shot drop / recovery tracking
    private boolean wasReady = false, inDrop = false;
    private double dropMinVel = 0, lastDrop = 0, lastRecoveryMs = 0;
    private int shots = 0;
    private final ElapsedTime dropTimer = new ElapsedTime();

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

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        batteryVoltage = voltageSensor.getVoltage();

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
            tele.addData("Controller", USE_CUSTOM ? "CUSTOM kS+kV+kP+kD" : "HUB PIDF");
            tele.addData("Custom", "kS %.3f  kV %.6f  kP %.4f  kD %.6f", kS, kV, kP, kD);
            tele.addData("Hub PIDF", "P %.3f  I %.3f  D %.3f  F %.3f", Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
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
        double actual = shooter1.getVelocity();

        // Battery voltage (filtered) for compensation
        batteryVoltage += Range.clip(VOLTAGE_FILTER, 0, 1) * (voltageSensor.getVoltage() - batteryVoltage);
        double voltageScale = (VOLTAGE_COMP && batteryVoltage > 6.0) ? NOMINAL_VOLTAGE / batteryVoltage : 1.0;

        // Switch motor mode when the controller choice changes
        if (lastUseCustom == null || lastUseCustom != USE_CUSTOM) {
            shooter1.setMode(USE_CUSTOM ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : DcMotor.RunMode.RUN_USING_ENCODER);
            if (!USE_CUSTOM) {
                lastP = Double.NaN; // force PIDF re-send
                applyPIDFIfChanged();
            }
            ff.reset();
            lastUseCustom = USE_CUSTOM;
        }

        if (USE_CUSTOM) {
            // RUN_WITHOUT_ENCODER still reads the encoder velocity.
            shooter1.setPower(ff.update(target, actual, kS, kV, kP, kD, D_FILTER,
                    BOOST_ENABLED, BOOST_ENTER, BOOST_EXIT, BOOST_POWER, voltageScale));
        } else {
            shooter1.setVelocity(target);
        }

        trackShots(target, actual);
        showTelemetry(target, actual, voltageScale);
    }

    /** Detects a velocity drop after being at speed (= a shot) and times the recovery. */
    private void trackShots(double target, double actual) {
        if (target <= 0) {
            wasReady = inDrop = false;
            return;
        }
        double err = target - actual;
        if (!inDrop) {
            if (wasReady && err > DROP_DETECT) {
                inDrop = true;
                dropMinVel = actual;
                dropTimer.reset();
            }
            wasReady = Math.abs(err) < READY_TOL;
        } else {
            dropMinVel = Math.min(dropMinVel, actual);
            if (Math.abs(err) < READY_TOL) {
                inDrop = false;
                wasReady = true;
                lastDrop = target - dropMinVel;
                lastRecoveryMs = dropTimer.milliseconds();
                shots++;
            }
        }
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

    private void showTelemetry(double target, double actual, double voltageScale) {

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
        double power = shooter1.getPower();
        tele.addData("PowerX1000", power * 1000); // same scale as velocity on the graph
        tele.addData("Ready", Math.abs(target - actual) < READY_TOL && target > 0);
        tele.addData("Controller", USE_CUSTOM ? "CUSTOM" : "HUB PIDF");
        if (USE_CUSTOM) {
            tele.addData("Boosting", ff.boosting ? 1 : 0);
            tele.addData("kS term", "%.3f", ff.ksTerm);
            tele.addData("kV term", "%.3f", ff.ffTerm);
            tele.addData("kP term", "%.3f", ff.pTerm);
            tele.addData("kD term", "%.3f", ff.dTerm);
            // kV that would hold this speed with zero error
            if (target > 0 && actual > 200 && Math.abs(target - actual) < READY_TOL && !ff.boosting) {
                tele.addData("kV estimate", "%.6f", (power / voltageScale - kS) / actual);
            }
        }
        tele.addData("Battery V", "%.2f  (scale %.3f)", batteryVoltage, voltageScale);
        tele.addData("Shots", shots);
        tele.addData("Last drop (ticks/s)", lastDrop);
        tele.addData("Last recovery ms", lastRecoveryMs);
        tele.addData("Current (A)", "%.2f", shooter1.getCurrent(CurrentUnit.AMPS));
        tele.addData("Intake", intakeState);
        tele.addLine(USE_CUSTOM
                ? "Tune order: kS -> kV -> kP -> kD -> BOOST (see file header)"
                : "Hub PIDF: tune F first (F ≈ 32767 / max ticks/s), then P.");
        tele.update();
    }
}
