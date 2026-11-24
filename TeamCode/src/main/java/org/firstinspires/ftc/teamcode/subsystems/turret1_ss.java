package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/*
 * turret1_ss - odometry turret aiming for turret1 (POLLEN, shooter 1) (Pedro v3 follower pose: Pinpoint pods + IMU).
 * New subsystem; does not change TrackingSS / ShooterSS.
 *
 * turret2 (Nectar, shooter 2) has its own copy: turret2_ss.
 *
 * ANGLES (turret, relative to robot front, seen from behind):
 *     0 deg = straight front      servo 0.50
 *   -90 deg = robot LEFT          servo 0.72
 *   +90 deg = robot RIGHT         servo 0.28
 *   Functional range -90 .. +90. Outside it the turret holds the nearest limit
 *   and telemetry shows AT LIMIT.
 *
 * FIELD (Pedro): (0,0) bottom-left, x right, y up, heading CCW+ in radians.
 *
 * GOALS: each alliance has two hives (UP / DOWN). Goal mode:
 *   AUTO       robot y > HALF_Y -> UP hive, else DOWN hive
 *   FORCE_UP   always UP hive   (button override)
 *   FORCE_DOWN always DOWN hive (button override)
 *
 * Usage every loop:  turret1.update(follower.pose());
 */
@Config
@Configurable
public class turret1_ss extends SubsystemBase {

    // ===== Turret1 (Pollen) calibration: two-point, piecewise linear =====
    public static double TURRET1_CENTER = 0.50;    // 0 deg
    public static double TURRET1_LEFT_90 = 0.72;   // -90 deg (robot left)
    public static double TURRET1_RIGHT_90 = 0.28;  // +90 deg (robot right)
    public static double TURRET1_MIN_DEG = -90;    // functional range
    public static double TURRET1_MAX_DEG = 90;
    public static double TURRET1_TRIM_DEG = 0;     // small aim correction, + = more right

    // Turret1 pivot offset from robot centre (inches). TODO measure.
    public static double TURRET1_OFFSET_FORWARD = 0; // + = towards robot front
    public static double TURRET1_OFFSET_LEFT = 0;    // + = towards robot left

    // ===== Goals (copied from Globals hive poses; tune here live) =====
    public static double RED_UP_X = Globals.redHiveUpSide.x();
    public static double RED_UP_Y = Globals.redHiveUpSide.y();
    public static double RED_DOWN_X = Globals.redHiveDownSide.x();
    public static double RED_DOWN_Y = Globals.redHiveDownSide.y();
    public static double BLUE_UP_X = Globals.blueHiveUpSide.x();
    public static double BLUE_UP_Y = Globals.blueHiveUpSide.y();
    public static double BLUE_DOWN_X = Globals.blueHiveDownSide.x();
    public static double BLUE_DOWN_Y = Globals.blueHiveDownSide.y();
    public static double HALF_Y = 70.75; // AUTO: above this -> UP hive

    // ===== Misc =====
    public static double WRITE_TOL = 0.002;       // skip tiny servo writes (less jitter, faster loop)
    public static double MAX_HEADING_JUMP = 0.5;  // rad per loop; bigger jump = bad reading, ignored

    public enum GoalMode { AUTO, FORCE_UP, FORCE_DOWN }
    public enum Hive { UP, DOWN }

    public boolean tracking = false;
    public GoalMode goalMode = GoalMode.AUTO;

    // Telemetry outputs
    public Hive activeHive = Hive.DOWN;
    public double goalX, goalY;
    public double distance;      // turret pivot -> goal (inches)
    public double wantedDeg;     // turret angle needed (before clipping)
    public double commandedDeg;  // turret angle actually sent (after clipping)
    public boolean atLimit = false;
    public boolean badPose = false;

    private final RobotHardware robot;
    private double target1 = TURRET1_CENTER;
    private double lastWrite1 = Double.NaN;
    private double lastHeading = Double.NaN;

    public turret1_ss(RobotHardware robot) {
        this.robot = robot;
    }

    // ================= main loop =================

    /** Call every loop with follower.pose(). */
    public void update(Pose pose) {
        double x = pose.x();
        double y = pose.y();
        double h = pose.heading();

        // Bad reading guard (NaN or sudden heading jump) -> keep last turret position.
        badPose = Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(h)
                || (!Double.isNaN(lastHeading) && Math.abs(wrap(h - lastHeading)) > MAX_HEADING_JUMP);
        if (!Double.isNaN(h)) lastHeading = h;
        if (badPose) return;

        selectGoal(y);

        // Turret pivot position on the field
        double cos = Math.cos(h), sin = Math.sin(h);
        double tx = x + TURRET1_OFFSET_FORWARD * cos - TURRET1_OFFSET_LEFT * sin;
        double ty = y + TURRET1_OFFSET_FORWARD * sin + TURRET1_OFFSET_LEFT * cos;

        double dx = goalX - tx, dy = goalY - ty;
        distance = Math.hypot(dx, dy);

        // Angle to goal relative to robot front. Math gives CCW+ (left +),
        // turret convention is right +, so flip the sign.
        double relLeftRad = wrap(Math.atan2(dy, dx) - h);
        wantedDeg = -Math.toDegrees(relLeftRad) + TURRET1_TRIM_DEG;

        commandedDeg = Range.clip(wantedDeg, TURRET1_MIN_DEG, TURRET1_MAX_DEG);
        atLimit = commandedDeg != wantedDeg;

        if (tracking) {
            target1 = degToPos1(commandedDeg);
            write1(target1);
        }
    }

    private void selectGoal(double robotY) {
        switch (goalMode) {
            case FORCE_UP:   activeHive = Hive.UP; break;
            case FORCE_DOWN: activeHive = Hive.DOWN; break;
            default:         activeHive = robotY > HALF_Y ? Hive.UP : Hive.DOWN; break;
        }
        boolean red = RobotHardware.isRed();
        if (activeHive == Hive.UP) {
            goalX = red ? RED_UP_X : BLUE_UP_X;
            goalY = red ? RED_UP_Y : BLUE_UP_Y;
        } else {
            goalX = red ? RED_DOWN_X : BLUE_DOWN_X;
            goalY = red ? RED_DOWN_Y : BLUE_DOWN_Y;
        }
    }

    // ================= conversions =================

    /** Turret1 degrees (right +) -> servo position. Separate slope each side. */
    public static double degToPos1(double deg) {
        double pos = deg >= 0
                ? TURRET1_CENTER + deg / 90.0 * (TURRET1_RIGHT_90 - TURRET1_CENTER)
                : TURRET1_CENTER + -deg / 90.0 * (TURRET1_LEFT_90 - TURRET1_CENTER);
        return clipPos1(pos);
    }

    /** Servo position -> turret1 degrees (right +). */
    public static double posToDeg1(double pos) {
        return (pos - TURRET1_CENTER) / (TURRET1_RIGHT_90 - TURRET1_CENTER) >= 0
                ? 90.0 * (pos - TURRET1_CENTER) / (TURRET1_RIGHT_90 - TURRET1_CENTER)
                : -90.0 * (pos - TURRET1_CENTER) / (TURRET1_LEFT_90 - TURRET1_CENTER);
    }

    private static double clipPos1(double pos) {
        double a = degToPosRaw(TURRET1_MIN_DEG), b = degToPosRaw(TURRET1_MAX_DEG);
        return Range.clip(pos, Math.min(a, b), Math.max(a, b));
    }

    private static double degToPosRaw(double deg) {
        return deg >= 0
                ? TURRET1_CENTER + deg / 90.0 * (TURRET1_RIGHT_90 - TURRET1_CENTER)
                : TURRET1_CENTER + -deg / 90.0 * (TURRET1_LEFT_90 - TURRET1_CENTER);
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    private void write1(double pos) {
        if (!Double.isNaN(lastWrite1) && Math.abs(pos - lastWrite1) < WRITE_TOL) return;
        robot.turret1.setPosition(pos);
        lastWrite1 = pos;
    }

    // ================= manual control =================

    /** Stop tracking and point turret1 at an angle (deg, right +). */
    public void setTurret1Deg(double deg) {
        tracking = false;
        target1 = degToPos1(deg);
        write1(target1);
    }

    public void center() {
        setTurret1Deg(0);
    }

    /** AUTO -> FORCE_UP -> FORCE_DOWN -> AUTO */
    public void cycleGoalMode() {
        goalMode = GoalMode.values()[(goalMode.ordinal() + 1) % GoalMode.values().length];
    }

    public double getTarget1() {
        return target1;
    }

    // ================= commands =================

    public Command trackCommand(boolean on) {
        return new InstantCommand(() -> {
            tracking = on;
            lastWrite1 = Double.NaN; // force next write
            if (!on) center();
        });
    }

    public Command centerCommand() {
        return new InstantCommand(this::center);
    }

    public Command goalModeCommand(GoalMode mode) {
        return new InstantCommand(() -> goalMode = mode);
    }
}
