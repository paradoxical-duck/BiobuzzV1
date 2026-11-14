package org.firstinspires.ftc.teamcode.scrimmage2.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Autonomous
public class RedUpLeave extends LinearOpMode {
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(62.2228, 132.4554, 90);
    private final Pose path1 = poseFactory.of(17.1077, 116.896, 0);
    private final Pose path1Control1 = poseFactory.of(51.3795, 103.3189, 0);

    public Path leave() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }
    boolean scheduled = false;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware.setAlliance(RobotHardware.Alliance.RED);
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            if (!scheduled) {
                schedule(autoRoutine(follower));
            }
            scheduled = true;

            Scheduler.execute();
        }
    }
    public Command autoRoutine(Follower follower) {
        return instant(() ->
                follower.follow(leave())
        );
    }
}
