package org.firstinspires.ftc.teamcode.scrimmage2.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class RedDownLeave extends LinearOpMode {
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.7778, 8.9723, 0);
    private final Pose path1 = poseFactory.of(14.3852, 90.0603, 0);
    private final Pose path1Control1 = poseFactory.of(16.8685, 35.7409, 0);
    boolean scheduled = false;
    public Path path1() {
        return Paths.curve(start, path1Control1, path1).constant(path1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            if (!scheduled) {
                schedule(instant(()-> follower.follow(path1())));
            }
            scheduled = true;
            Scheduler.execute();
        }
    }
}
