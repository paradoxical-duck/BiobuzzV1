package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
    public class pathV1test extends LinearOpMode {

        private Follower follower;

        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose start = poseFactory.of(32, 9, 180);
        private final Pose shoot1 = poseFactory.of(13, 9, 180);
        private final Pose shoot2Start = poseFactory.of(13, 9, 180);
        private final Pose shoot2 = poseFactory.of(37.3926, 118.6015, 90);
        private final Pose shoot2Control1 = poseFactory.of(24, 110, 0);
        private final Pose flower1Start = poseFactory.of(37.3926, 118.6015, 90);
        private final Pose flower1 = poseFactory.of(46.9856, 130.1902, 90);
        private final Pose parkStart = poseFactory.of(46.9856, 130.1902, 90);
        private final Pose park = poseFactory.of(10.063, 117.0475, 90);
        private final Pose parkControl1 = poseFactory.of(39.3444, 114.5848, 0);

        // Autonomous routine
        public Command autoRoutine() {
            return sequential(
                    follow(follower, shoot1()),
//                    waitMs(1000),
                    follow(follower, shoot2()),
                    follow(follower, flower1()),
                    follow(follower, park())
            );
        }

        @Override
        public void runOpMode() {
            Scheduler.reset();
            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            follower.update();

            waitForStart();
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();

                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", follower.pose().heading());

                if (follower.currentPath() != null) {
                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                    telemetry.addData("Path number", follower.pathIndex());
                }

                telemetry.update();
            }
        }

        public Path shoot1() {
            return Paths.line(start, shoot1).linear(start, shoot1);
        }

        public Path shoot2() {
            return Paths.curve(shoot2Start, shoot2Control1, shoot2).linear(shoot2Start, shoot2);
        }

        public Path flower1() {
            return Paths.line(flower1Start, flower1).linear(flower1Start, flower1);
        }

        public Path park() {
            return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
        }
    }
