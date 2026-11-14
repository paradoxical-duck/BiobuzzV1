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
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class BlueDownLeave extends LinearOpMode {
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(78.9465, 9.1668, 90);
    private final Pose path1 = poseFactory.of(126.0062, 28.6105, 0);
    private final Pose path1Control1 = poseFactory.of(94.9047, 32.5449, 0);

    public Path leave() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }  boolean scheduled = false;
    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        RobotHardware.setAlliance(RobotHardware.Alliance.BLUE);
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
