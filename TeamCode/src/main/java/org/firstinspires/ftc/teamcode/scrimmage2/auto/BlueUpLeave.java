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
public class BlueUpLeave extends LinearOpMode {
    Follower follower;
    PoseFactory poseFactory = PoseFactory.degrees();
    public final Pose startPose = poseFactory.of(78.4765, 132.0326, 180);
    private final Pose point7 = poseFactory.of(128.1012, 48.793, 82.3947);
    private final Pose point7Control1 = poseFactory.of(122.7862, 90.0553, 180);
    public Path leave() {
        return Paths.curve(startPose, point7Control1, point7).tangent();
    }
    boolean scheduled = false;
    @Override
    public void runOpMode() throws InterruptedException {
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
