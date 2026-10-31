package org.firstinspires.ftc.teamcode.VersionTesting;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Side;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSS;

@TeleOp(group = "Version Testing")
@Configurable
public class TrackingTestV1 extends LinearOpMode {
    TrackingSS trackingSS;
    RobotHardware robot = RobotHardware.getInstance();
    Follower follower;
    Side.ALLIANCE alliance = Side.ALLIANCE.RED;
    boolean lastLB = false;
    boolean red = true;
    boolean lastStart = false;
    boolean lastBack = false;
    boolean lastRB = false;
    public static boolean trackingEnabled = false, interpolEnabled = false, shooterEnabled = false;
    ShooterSS shooterSS;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        trackingSS = new TrackingSS(robot, follower, alliance);
        shooterSS = new ShooterSS(robot);

        while (opModeInInit()) {
            if (gamepad1.left_bumper && !lastLB) {
                red = !red;
            }
            lastLB = gamepad1.left_bumper;
        }
        while (opModeIsActive()) {
            // todo drive and see interpolation
            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            follower.update();

            if (trackingEnabled) {
                TrackingSS.updateTracking(shooterSS);

                shooterSS.updateState(ShooterSS.TurretState.TRACKING_ENABLED);
                shooterSS.updateState(ShooterSS.HoodState.INTERPOLATED);

                if (shooterEnabled){
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOLATED);
                }
            }
            else {
                shooterSS.updateState(ShooterSS.TurretState.TRACKING_DISABLED);
                shooterSS.updateState(ShooterSS.HoodState.SHOOT);

                if (shooterEnabled) {
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOL_DISABLED);
                }
            }

            if (gamepad1.start && !lastStart) {
                trackingEnabled = !trackingEnabled;
            }
            lastStart = gamepad1.start;

            if (gamepad1.back && !lastBack) {
                interpolEnabled = !interpolEnabled;
            }
            lastBack = gamepad1.back;

            if (gamepad1.right_bumper && !lastRB) {

            }


        }
    }
}
