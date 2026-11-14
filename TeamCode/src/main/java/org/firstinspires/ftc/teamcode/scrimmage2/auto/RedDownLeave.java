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
public class RedDownLeave extends LinearOpMode {
    Follower follower;
    PoseFactory poseFactory = PoseFactory.degrees();
    public final Pose startPose = poseFactory.of(63.0235, 9.4674, 180);
    private final Pose point7 = poseFactory.of(13.3988, 92.707, 97.6053);
    private final Pose point7Control1 = poseFactory.of(18.7138, 51.4447, 0);
    public Path leave() {
        return Paths.curve(startPose, point7Control1, point7).tangent();
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
