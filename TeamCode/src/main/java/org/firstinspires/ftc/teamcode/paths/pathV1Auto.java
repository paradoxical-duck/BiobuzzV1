package org.firstinspires.ftc.teamcode.paths;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.driveCommand.PathCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * FTCLib version of pathV1test: same pathV1 paths, run with PathCommand.
 *
 * initialize() runs once at Init, run() runs every loop after Start.
 * CommandOpMode calls waitForStart() and resets the CommandScheduler at the end.
 */
@Autonomous(name = "AutoPath FTCLib", group = "Autonomous")
public class pathV1Auto extends CommandOpMode {

    private Follower follower;
    private pathV1 path;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        path = new pathV1();
        follower = Constants.create(hardwareMap);
        follower.setPose(path.start);
        follower.update();

        //todo add subsystem commands (InstantCommand / WaitUntilCommand) between paths
        schedule(new SequentialCommandGroup(
                new PathCommand(follower, path.shoot1()),
                new WaitCommand(1000),
                new PathCommand(follower, path.shoot2()),
                new PathCommand(follower, path.flower1()),
                new PathCommand(follower, path.park())
        ));

        while (opModeInInit()) {
            telemetry.addLine("Ready");
            telemetry.addData("Start", "x %.1f  y %.1f  h %.1f",
                    follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }
    }

    @Override
    public void run() {
        super.run(); // CommandScheduler.run()
        // = CommandScheduler.getInstance().run()
        follower.update();

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("busy", follower.isBusy());

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }

        telemetry.update();
    }
}
