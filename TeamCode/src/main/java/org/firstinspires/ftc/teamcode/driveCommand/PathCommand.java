package org.firstinspires.ftc.teamcode.driveCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.api.Paths.line;

public class PathCommand extends CommandBase {

    private final Follower follower;
    private final Path path;

    private final boolean holdAtEnd;
    private final String side;
    private final double timeoutSeconds;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean started = false;

    public PathCommand(Follower follower, Path path) {
        this(follower, path, true, "null", Double.POSITIVE_INFINITY);
    }

    public PathCommand(Follower follower, Path path, double timeoutSeconds) {
        this(follower, path, true, "null", timeoutSeconds);
    }

    public PathCommand(Follower follower, Path path, String side) {
        this(follower, path, true, side, Double.POSITIVE_INFINITY);
    }

    public PathCommand(Follower follower, Path path, String side, double timeoutSeconds) {
        this(follower, path, false, side, timeoutSeconds);
    }

    public PathCommand(
            Follower follower,
            Path path,
            boolean holdAtEnd,
            String side,
            double timeoutSeconds
    ) {
        this.follower = follower;
        this.path = path;
        this.holdAtEnd = holdAtEnd;
        this.side = side;
        this.timeoutSeconds = timeoutSeconds;

        addRequirements();
    }

    @Override
    public void initialize() {
        timer.reset();
        follower.follow(path);
        started = true;
    }

    @Override
    public boolean isFinished() {
        return started &&
                (follower.atParametricEnd() || timer.seconds() >= timeoutSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        boolean timedOut = timer.seconds() >= timeoutSeconds;

        if (timedOut) {
            follower.hold(follower.pose());
            return;
        }

        if (!holdAtEnd && !interrupted && side.equalsIgnoreCase("RED")) {
            PedroHold.holdRed(follower);
        } else if (!holdAtEnd && !interrupted && side.equalsIgnoreCase("BLUE")) {
            PedroHold.holdBlue(follower);
        } else if (holdAtEnd && !interrupted) {
            follower.hold(follower.pose());
        } else {
            follower.hold(follower.pose());
        }
    }

    public static class PedroHold {

        public static void holdRed(Follower follower) {
            Pose p = follower.pose();
            follower.follow(line(p, new Pose(135, 62)).heading(Interpolator.constant(p.heading())));
        }

        public static void holdBlue(Follower follower) {
            Pose p = follower.pose();
            follower.follow(line(p, new Pose(17.5, 60)).heading(Interpolator.constant(p.heading())));
        }
    }
}
