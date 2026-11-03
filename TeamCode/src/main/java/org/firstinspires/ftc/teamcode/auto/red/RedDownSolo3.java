
package org.firstinspires.ftc.teamcode.auto.red;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.paths.RedDownSolo3Path;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

@Autonomous
//todo before auto: GET VALUES FOR AUTO SHOOT TURRET AND HOOD
public class RedDownSolo3 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();

    Follower follower;
    RedDownSolo3Path path;
    boolean scheduled = false;
    IntakeSS intakeSS;
    ShooterSS shooterSS;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        path = new RedDownSolo3Path();
        follower = Constants.create(hardwareMap);
        follower.setPose(path.startPose);

        while (opModeInInit()) {
            Scheduler.reset();
        }
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            if (!scheduled) {
                schedule(path.autoRoutine(follower, intakeSS, shooterSS));
            }
            scheduled = true;

            Scheduler.execute();

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Follower Mode", follower.mode());
            telemetry.update();
        }
    }
}
