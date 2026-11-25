package org.firstinspires.ftc.teamcode.versionTesting.currentTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Drive + intake motor CURRENT test.
 *
 * Shows live, peak and average current (amps) for the 4 drive motors and
 * intake1 / intake2, plus totals. Use it to find binding / tight drivetrain
 * corners, jammed intake rollers, or a motor that draws much more than its twin.
 *
 * Gamepad1:
 *   Left stick       drive forward / strafe
 *   Right stick X    turn
 *   RIGHT BUMPER     hold: intake inward
 *   LEFT BUMPER      hold: intake outward
 *   A                reset peaks and averages
 *   Y                drive scale 100% <-> 50%
 *
 * Dashboard Graph (192.168.43.1:8080/dash): LF, RF, LB, RB, Intake1, Intake2,
 * DriveTotal, IntakeTotal (amps).
 *
 * Note: each current read is a separate hub command (not bulk cached), ~1-2 ms
 * each. READ_EVERY_N_LOOPS > 1 reads less often if the loop gets slow.
 */
@Config
@Configurable
@TeleOp(name = "Drive Intake Current Test", group = "Version Testing")
public class DriveIntakeCurrentTest extends CommandOpMode {

    public static double DRIVE_SCALE = 1.0;
    public static double TURN_SCALE = 0.7;
    public static double INTAKE_POWER = 1.0;      // uses Globals sign for direction
    public static int READ_EVERY_N_LOOPS = 1;
    public static double WARN_DRIVE_AMPS = 5.0;   // flag a drive motor above this
    public static double WARN_INTAKE_AMPS = 4.0;  // flag an intake motor above this

    private static final String[] NAMES = {"LF", "RF", "LB", "RB", "Intake1", "Intake2"};

    private final RobotHardware robot = RobotHardware.getInstance();
    private Follower follower;
    private Telemetry tele;
    private DcMotorEx[] motors;

    private final double[] amps = new double[6];
    private final double[] peak = new double[6];
    private final double[] sum = new double[6];
    private long samples = 0;

    private final Gamepad cur = new Gamepad();
    private final Gamepad prev = new Gamepad();
    private boolean halfSpeed = false;
    private int loops = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.update();

        motors = new DcMotorEx[]{
                hardwareMap.get(DcMotorEx.class, "left front"),
                hardwareMap.get(DcMotorEx.class, "right front"),
                hardwareMap.get(DcMotorEx.class, "left back"),
                hardwareMap.get(DcMotorEx.class, "right back"),
                robot.intake1,
                robot.intake2
        };

        while (opModeInInit()) {
            tele.addLine("Drive + intake current test ready.");
            tele.addLine("Sticks drive | RB intake in | LB out | A reset peaks | Y half speed");
            tele.update();
        }
        loopTimer.reset();
    }

    @Override
    public void run() {
        robot.clearBulkCache();
        super.run();

        prev.copy(cur);
        cur.copy(gamepad1);

        if (cur.a && !prev.a) resetStats();
        if (cur.y && !prev.y) halfSpeed = !halfSpeed;

        // ---------------- Drive ----------------
        double scale = DRIVE_SCALE * (halfSpeed ? 0.5 : 1.0);
        follower.manual(
                -cur.left_stick_y * scale,
                -cur.left_stick_x * scale,
                -cur.right_stick_x * TURN_SCALE * scale
        );
        follower.update();

        // ---------------- Intake ----------------
        double intakePower = 0;
        if (cur.right_bumper) intakePower = Math.abs(INTAKE_POWER) * Math.signum(Globals.intakeOn);
        else if (cur.left_bumper) intakePower = Math.abs(INTAKE_POWER) * Math.signum(Globals.intakeRev);
        robot.intake1.setPower(intakePower);
        robot.intake2.setPower(intakePower);

        // ---------------- Current ----------------
        loops++;
        if (loops % Math.max(1, READ_EVERY_N_LOOPS) == 0) {
            for (int i = 0; i < motors.length; i++) {
                amps[i] = motors[i].getCurrent(CurrentUnit.AMPS);
                peak[i] = Math.max(peak[i], amps[i]);
                sum[i] += amps[i];
            }
            samples++;
        }

        double driveTotal = amps[0] + amps[1] + amps[2] + amps[3];
        double intakeTotal = amps[4] + amps[5];

        // ---------------- Telemetry ----------------
        tele.addData("Loop ms", "%.1f", loopTimer.milliseconds());
        loopTimer.reset();
        tele.addData("Drive speed", halfSpeed ? "50%" : "100%");
        tele.addData("Intake power", intakePower);
        tele.addData("Battery V", "%.2f", robot.voltageSensor.getVoltage());
        tele.addLine("------ amps: now | peak | avg ------");
        for (int i = 0; i < motors.length; i++) {
            double warn = i < 4 ? WARN_DRIVE_AMPS : WARN_INTAKE_AMPS;
            tele.addData(NAMES[i] + (amps[i] > warn ? "  !! HIGH" : ""),
                    "%.2f | %.2f | %.2f", amps[i], peak[i], samples > 0 ? sum[i] / samples : 0);
        }
        tele.addLine("------ totals ------");
        tele.addData("Drive total (A)", "%.2f", driveTotal);
        tele.addData("Intake total (A)", "%.2f", intakeTotal);
        tele.addData("Left vs right drive (A)", "%.2f | %.2f", amps[0] + amps[2], amps[1] + amps[3]);
        tele.addData("Intake1 vs Intake2 (A)", "%.2f | %.2f", amps[4], amps[5]);

        // Plain numbers for the Dashboard Graph tab
        tele.addLine("------ graph (A) ------");
        for (int i = 0; i < motors.length; i++) tele.addData(NAMES[i], amps[i]);
        tele.addData("DriveTotal", driveTotal);
        tele.addData("IntakeTotal", intakeTotal);

        tele.update();
    }

    private void resetStats() {
        for (int i = 0; i < peak.length; i++) {
            peak[i] = 0;
            sum[i] = 0;
        }
        samples = 0;
    }
}
