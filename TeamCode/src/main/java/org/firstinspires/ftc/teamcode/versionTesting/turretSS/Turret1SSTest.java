package org.firstinspires.ftc.teamcode.versionTesting.turretSS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret1_ss;

/*
 * Test for turret1_ss (turret1 / Pollen odometry aiming).
 *
 * Place the robot at START_X / START_Y / START_HEADING_DEG before INIT
 * (defaults: bottom middle of the field, facing up). Aiming is only as good
 * as the starting pose.
 *
 * INIT:
 *   RB / LB        alliance RED / BLUE
 *   A              field centric on/off
 * MATCH (gamepad1):
 *   sticks         drive
 *   DPAD UP        tracking on/off
 *   DPAD RIGHT     goal mode AUTO -> FORCE_UP -> FORCE_DOWN
 *   DPAD DOWN      stop tracking, turret to centre
 *   B              reset pose to START pose (robot back on start spot)
 *   RIGHT STICK BTN  reset pose to corner (by alliance, drive-team side):
 *                      RED  -> (9, 9, 0)         BLUE -> (132.5, 132.5, 180)
 *   LEFT STICK BTN   reset pose to corner:
 *                      RED  -> (9, 132.5, 0)     BLUE -> (132.5, 9, 180)
 *   Push the robot into that corner first, facing the given heading.
 *
 * FIELD CENTRIC is from the DRIVER's view: stick forward = away from your
 * drive-team wall. RED drivers stand at x = 0 (forward = +x),
 * BLUE drivers at x = 141.5 (forward = -x) -> RED_DRIVER_DEG / BLUE_DRIVER_DEG.
 *
 * Live values: Dashboard Config -> turret1_ss / Turret1SSTest (click SAVE),
 *              or Panels Configurables.
 */
@Config
@Configurable
@TeleOp(name = "Turret1 SS Test", group = "Version Testing")
public class Turret1SSTest extends CommandOpMode {

    public static double START_X = 9;
    public static double START_Y = 9;
    public static double START_HEADING_DEG = 0;
    public static boolean FIELD_CENTRIC = true;

    // Corner reset poses (x, y in inches, heading in degrees). Tune live.
    public static double RED_RS_X = 9,      RED_RS_Y = 9,      RED_RS_H = 0;    // RED  right stick btn
    public static double RED_LS_X = 9,      RED_LS_Y = 132.5,  RED_LS_H = 0;    // RED  left stick btn
    public static double BLUE_RS_X = 132.5, BLUE_RS_Y = 132.5, BLUE_RS_H = 180; // BLUE right stick btn
    public static double BLUE_LS_X = 132.5, BLUE_LS_Y = 9,     BLUE_LS_H = 180; // BLUE left stick btn

    // Field-centric "forward" direction for each drive team (field degrees).
    public static double RED_DRIVER_DEG = 0;    // red drivers look towards +x
    public static double BLUE_DRIVER_DEG = 180; // blue drivers look towards -x

    private String lastReset = "start pose";

    private final RobotHardware robot = RobotHardware.getInstance();
    private Follower follower;
    private turret1_ss turret;
    private GamepadEx gp;
    private Telemetry tele;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose());
        follower.update();

        turret = new turret1_ss(robot);

        gp = new GamepadEx(gamepad1);
        gp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> schedule(turret.trackCommand(!turret.tracking))));
        gp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(turret::cycleGoalMode));
        gp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(turret.centerCommand());
        gp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    follower.setPose(startPose());
                    lastReset = "start pose";
                }));
        gp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> cornerReset(true)));
        gp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> cornerReset(false)));

        boolean lastA = false;
        while (opModeInInit()) {
            if (gamepad1.right_bumper) RobotHardware.setAlliance(RobotHardware.Alliance.RED);
            if (gamepad1.left_bumper) RobotHardware.setAlliance(RobotHardware.Alliance.BLUE);
            if (gamepad1.a && !lastA) FIELD_CENTRIC = !FIELD_CENTRIC;
            lastA = gamepad1.a;

            tele.addLine("RB red | LB blue | A field centric");
            tele.addData("Alliance", RobotHardware.getAlliance());
            tele.addData("Field centric", FIELD_CENTRIC);
            tele.addData("Start pose", "(%.1f, %.1f, %.0f deg)", START_X, START_Y, START_HEADING_DEG);
            tele.update();
        }
        loopTimer.reset();
    }

    @Override
    public void run() {
        robot.clearBulkCache();
        super.run();

        // ---------------- Drive ----------------
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x * 0.7;
        if (FIELD_CENTRIC) {
            // Subtract the driver's view direction so stick forward = away from the driver.
            double driverDeg = RobotHardware.isRed() ? RED_DRIVER_DEG : BLUE_DRIVER_DEG;
            DrivePowers powers = ManualDrive.fieldCentric(forward, strafe, turn,
                    follower.pose().heading() - Math.toRadians(driverDeg));
            follower.manual(powers);
        } else {
            follower.manual(forward, strafe, turn);
        }
        follower.update();

        // ---------------- Turret ----------------
        turret.update(follower.pose());

        // ---------------- Telemetry ----------------
        Pose p = follower.pose();
        tele.addData("Loop ms", "%.1f", loopTimer.milliseconds());
        loopTimer.reset();
        tele.addLine("DPAD UP track | DPAD RIGHT goal mode | DPAD DOWN centre | B reset pose");
        tele.addLine("RS btn / LS btn: corner pose reset (by alliance)");
        tele.addData("Last pose reset", lastReset);
        tele.addData("Alliance", RobotHardware.getAlliance());
        tele.addData("Tracking", turret.tracking);
        tele.addData("Goal mode", turret.goalMode);
        tele.addData("Active hive", turret.activeHive);
        tele.addData("Goal", "(%.1f, %.1f)", turret.goalX, turret.goalY);
        tele.addData("Distance (in)", "%.1f", turret.distance);
        tele.addData("Wanted deg", "%.1f", turret.wantedDeg);
        tele.addData("Commanded deg", "%.1f", turret.commandedDeg);
        tele.addData("Turret1 servo", "%.3f", turret.getTarget1());
        tele.addData("AT LIMIT", turret.atLimit);
        tele.addData("Bad pose (ignored)", turret.badPose);
        tele.addLine("------------------------------");
        tele.addData("x", "%.1f", p.x());
        tele.addData("y", "%.1f", p.y());
        tele.addData("heading deg", "%.1f", Math.toDegrees(p.heading()));
        tele.update();
    }

    /** Sets the pose to one of the two corners of the current alliance. */
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
        lastReset = String.format("%s %s (%.1f, %.1f, %.0f)",
                red ? "RED" : "BLUE", rightStick ? "RS" : "LS", x, y, h);
    }

    private static Pose startPose() {
        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }
}
