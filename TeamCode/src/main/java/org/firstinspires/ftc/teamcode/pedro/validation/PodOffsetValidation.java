package org.firstinspires.ftc.teamcode.pedro.validation;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Locale;

/**
 * Validates the Pinpoint xPodOffset / yPodOffset in Constants.
 * Nothing is changed automatically: copy the suggested average into Constants yourself.
 *
 * The robot spins itself TURNS turns in place, TRIALS times (alternating spin direction).
 * If the offsets are right, the reported x/y stays at 0. If not, it traces a circle,
 * and fitting that circle gives the correction. The fit also solves for the robot
 * sliding across the floor while spinning (start offset + drift per turn) and ignores it.
 *
 * Every trial and the final average are appended to /sdcard/FIRST/pod_offsets.csv
 *
 * Controls (gamepad1):
 *   A  start next trial (robot must be still, away from walls)
 *   B  stop the running trial
 *   Y  clear results and start over
 */
@TeleOp(name = "Pod Offset Validation", group = "Validation")
public class PodOffsetValidation extends LinearOpMode {
    public static int TURNS = 5;
    public static int TRIALS = 3;
    public static double SPIN_POWER = 0.25;

    private Follower follower;
    private final File logFile = new File(AppUtil.ROOT_FOLDER, "pod_offsets.csv");

    private double xOffsetInUse, yOffsetInUse;
    private double totalHeading, prevHeading;

    private boolean running = false;
    private int spinDirection = -1;
    private boolean lastA, lastB, lastY;

    // Least squares normal equations for [tx, ty, ax, bx, ay, by], see spinStep().
    private final double[][] normal = new double[6][6];
    private final double[] rhs = new double[6];
    private double sumSq, maxWander;
    private long samples;

    private final double[] trialX = new double[TRIALS];
    private final double[] trialY = new double[TRIALS];
    private int trialsDone = 0;
    private String lastResult = "none yet";

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        xOffsetInUse = Constants.localizerConfig.xPodOffset.get();
        yOffsetInUse = Constants.localizerConfig.yPodOffset.get();

        telemetry.addLine("Pod Offset Validation: " + TRIALS + " trials x " + TURNS + " turns");
        telemetry.addData("Offsets in use", "X %.4f  Y %.4f in", xOffsetInUse, yOffsetInUse);
        telemetry.addLine("Put the robot in open space. Press start, then A for each trial.");
        telemetry.update();
        waitForStart();

        zero();
        while (opModeIsActive()) {
            follower.update();
            trackHeading();

            if (running) {
                spinStep();
            } else {
                follower.manual(0, 0, 0);
            }
            handleButtons();
            showTelemetry();
        }
        follower.manual(0, 0, 0);
        follower.update();
    }

    private void handleButtons() {
        boolean a = gamepad1.a, b = gamepad1.b, y = gamepad1.y;

        if (!running) {
            if (a && !lastA && trialsDone < TRIALS) startTrial();
            if (y && !lastY) {
                trialsDone = 0;
                lastResult = "cleared";
            }
        } else if (b && !lastB) {
            running = false;
            follower.manual(0, 0, 0);
            lastResult = "trial " + (trialsDone + 1) + " stopped, not counted";
        }

        lastA = a;
        lastB = b;
        lastY = y;
    }

    private void startTrial() {
        zero();
        for (double[] row : normal) Arrays.fill(row, 0);
        Arrays.fill(rhs, 0);
        sumSq = maxWander = 0;
        samples = 0;
        spinDirection = -spinDirection;
        running = true;
    }

    /** Called every loop while the robot spins itself. */
    private void spinStep() {
        follower.manual(0, 0, SPIN_POWER * spinDirection);

        // Reported pose of the tracking point while spinning:
        //   p = (R(theta) - I) * t + a + b * turns
        // t: tracking point's offset from the rotation center (robot frame) -> what we want.
        // a, b: the rotation center sliding across the floor (start offset and drift per turn) -> ignored.
        Pose p = follower.pose();
        double c = Math.cos(totalHeading) - 1, s = Math.sin(totalHeading);
        double turns = totalHeading / (2 * Math.PI);
        addRow(new double[]{c, -s, 1, turns, 0, 0}, p.x());
        addRow(new double[]{s, c, 0, 0, 1, turns}, p.y());
        maxWander = Math.max(maxWander, Math.hypot(p.x(), p.y()));

        if (Math.abs(totalHeading) >= TURNS * 2 * Math.PI) {
            running = false;
            follower.manual(0, 0, 0);
            finishTrial();
        }
    }

    private void addRow(double[] row, double value) {
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) normal[i][j] += row[i] * row[j];
            rhs[i] += row[i] * value;
        }
        sumSq += value * value;
        samples++;
    }

    /** Solves a * x = b by Gaussian elimination with partial pivoting. */
    private static double[] solve(double[][] a, double[] b) {
        int n = b.length;
        double[][] m = new double[n][n + 1];
        for (int i = 0; i < n; i++) {
            System.arraycopy(a[i], 0, m[i], 0, n);
            m[i][n] = b[i];
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col + 1; r < n; r++) if (Math.abs(m[r][col]) > Math.abs(m[pivot][col])) pivot = r;
            double[] tmp = m[col];
            m[col] = m[pivot];
            m[pivot] = tmp;
            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double f = m[r][col] / m[col][col];
                for (int k = col; k <= n; k++) m[r][k] -= f * m[col][k];
            }
        }
        double[] x = new double[n];
        for (int i = 0; i < n; i++) x[i] = m[i][n] / m[i][i];
        return x;
    }

    private void finishTrial() {
        double[] fit = solve(normal, rhs);
        double tx = fit[0], ty = fit[1];
        double driftPerTurn = Math.hypot(fit[3], fit[5]);
        double residualSq = sumSq;
        for (int i = 0; i < 6; i++) residualSq -= fit[i] * rhs[i];
        double rms = Math.sqrt(Math.max(residualSq, 0) / Math.max(samples, 1));

        // Moving the tracking point onto the rotation center shifts both pod offsets by t.
        double newX = xOffsetInUse + ty;
        double newY = yOffsetInUse + tx;
        trialX[trialsDone] = newX;
        trialY[trialsDone] = newY;
        trialsDone++;

        String dir = spinDirection > 0 ? "+turn" : "-turn";
        lastResult = String.format(Locale.US, "trial %d (%s): X %.4f  Y %.4f  (wander %.3f in, rms %.3f in)",
                trialsDone, dir, newX, newY, maxWander, rms);
        log("trial " + trialsDone, dir, newX, newY, ty, tx, maxWander, driftPerTurn, rms);

        if (trialsDone == TRIALS) {
            log("AVERAGE", "-", averageX(), averageY(), averageX() - xOffsetInUse, averageY() - yOffsetInUse, 0, 0, 0);
        }
    }

    private double averageX() {
        double sum = 0;
        for (int i = 0; i < trialsDone; i++) sum += trialX[i];
        return sum / trialsDone;
    }

    private double averageY() {
        double sum = 0;
        for (int i = 0; i < trialsDone; i++) sum += trialY[i];
        return sum / trialsDone;
    }

    private void log(String trial, String dir, double newX, double newY, double corrX, double corrY,
                     double wander, double drift, double rms) {
        boolean newFile = !logFile.exists();
        try (PrintWriter out = new PrintWriter(new FileWriter(logFile, true))) {
            if (newFile) {
                out.println("time,trial,direction,turns,x_offset_in_use,y_offset_in_use,"
                        + "suggested_x,suggested_y,correction_x,correction_y,wander_in,drift_in_per_turn,fit_rms_in");
            }
            String time = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date());
            out.printf(Locale.US, "%s,%s,%s,%d,%.4f,%.4f,%.4f,%.4f,%+.4f,%+.4f,%.3f,%.3f,%.3f%n",
                    time, trial, dir, TURNS, xOffsetInUse, yOffsetInUse,
                    newX, newY, corrX, corrY, wander, drift, rms);
        } catch (IOException e) {
            lastResult += " (NOT SAVED: " + e.getMessage() + ")";
        }
    }

    private void showTelemetry() {
        Pose p = follower.pose();
        telemetry.addLine("=== Pod Offset Validation ===");
        telemetry.addData("Offsets in use", "X %.4f  Y %.4f in", xOffsetInUse, yOffsetInUse);
        telemetry.addData("Pose", "x %.2f  y %.2f  h %.1f°", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addLine();

        if (running) {
            telemetry.addLine("RUNNING trial " + (trialsDone + 1) + "/" + TRIALS + "  (" + (spinDirection > 0 ? "+turn" : "-turn") + ")");
            telemetry.addData("Turns", "%.2f / %d", Math.abs(totalHeading) / (2 * Math.PI), TURNS);
            telemetry.addData("Wander so far", "%.3f in", maxWander);
            telemetry.addLine("B = stop");
        } else if (trialsDone < TRIALS) {
            telemetry.addLine("A = start trial " + (trialsDone + 1) + "/" + TRIALS + "   Y = clear");
        } else {
            telemetry.addLine("DONE. Copy the average into Constants.   Y = start over");
        }

        telemetry.addLine();
        telemetry.addData("Last result", lastResult);
        for (int i = 0; i < trialsDone; i++) {
            telemetry.addData("Trial " + (i + 1), "X %.4f  Y %.4f", trialX[i], trialY[i]);
        }
        if (trialsDone > 0) {
            telemetry.addData("AVERAGE", "X %.4f  Y %.4f  (over %d)", averageX(), averageY(), trialsDone);
        }
        telemetry.addData("Log", logFile.getPath());
        telemetry.update();
    }

    private void trackHeading() {
        double heading = follower.pose().heading();
        totalHeading += normalize(heading - prevHeading);
        prevHeading = heading;
    }

    private static double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void zero() {
        follower.setPose(new Pose(0, 0, 0));
        totalHeading = 0;
        prevHeading = 0;
    }
}
