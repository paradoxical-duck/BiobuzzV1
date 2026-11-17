package org.firstinspires.ftc.teamcode.scrimmage3.auto.red;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.scrimmage3.auto.paths.red.RedDownSolo2Path;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

@Autonomous(name = "Red Down Solo")
public class RedDownSolo2 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    Follower follower;
    RedDownSolo2Path path;
    boolean scheduled = false;
    IntakeSS intakeSS;
    ShooterSS shooterSS;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        RobotHardware.setAlliance(RobotHardware.Alliance.RED);
        intakeSS = new IntakeSS(robot);
        shooterSS = new ShooterSS(robot);

        path = new RedDownSolo2Path();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoConstants.redDownStartPose);

        Scheduler.reset();
        waitForStart();

        while (opModeIsActive()) {
            robot.clearBulkCache();
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