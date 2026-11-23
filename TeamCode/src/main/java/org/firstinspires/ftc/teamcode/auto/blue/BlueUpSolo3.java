
package org.firstinspires.ftc.teamcode.auto.blue;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

@Disabled
@Autonomous
//todo before auto: GET VALUES FOR AUTO SHOOT TURRET AND HOOD
public class BlueUpSolo3 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    Follower follower;
    AutoConstants.BlueUpSolo3Path path;
    boolean scheduled = false;
    IntakeSS intakeSS;
    ShooterSS shooterSS;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        RobotHardware.setAlliance(RobotHardware.Alliance.BLUE);
        intakeSS = new IntakeSS(robot);
        shooterSS = new ShooterSS(robot);

        path = new AutoConstants.BlueUpSolo3Path();
        follower = Constants.create(hardwareMap);
        follower.setPose(path.startPose);


        Scheduler.reset();
        waitForStart();

        while (opModeIsActive()) {
            robot.clearBulkCache();

            if (!scheduled) {
                schedule(path.autoRoutine(follower/*, intakeSS, shooterSS*/));
                scheduled = true;
            }

            Scheduler.execute();
            follower.update();

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Follower Mode", follower.mode());
            telemetry.update();
        }
    }
}
