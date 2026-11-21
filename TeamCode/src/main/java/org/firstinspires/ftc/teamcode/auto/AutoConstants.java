package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

public class AutoConstants {
    //*todo this is a class with constant auto poses and paths for consistency across paths
    public static PoseFactory poseFactory = PoseFactory.degrees();

    //*TODO RED DOWN ALLIANCE POSES
    public static Pose redDownStartPose = poseFactory.of(55, 9, 180);
    public static Pose redGardenPose = poseFactory.of(12, 9, 180);
    public static Pose redAllianceAdjFlower = poseFactory.of(10.7389, 37.7114, 111.3137);
    //*TODO flower pose for flower on side adjacent to red down start pose, when reaching flower from down side (NOT same as solo auto)
    public static Pose redAllianceAdjFlowerCtrl = poseFactory.of(19.7874, 14.5196, 0);
    public static Pose redDownLeave = poseFactory.of(13.3988, 92.707, 97.6053);
    public static Pose redDownLeaveCtrl = poseFactory.of(18.7138, 51.4447, 0);

    //*TODO RED SOLO (2) POSES
    public static Pose redFlowerAndShoot = poseFactory.of(53.85, 128.0821, 90);
    public static Pose redFlowerAndShootCtrl = poseFactory.of(45.0655, 61.4432, 0);
    public static Pose redFlowerAndShootCtrl2 = poseFactory.of( 0, 94.659, 0);
    public static Pose redSolo2Leave = poseFactory.of(13.2197, 116.5202, 90);

//    public static Pose flowerAndShoot = redFlowerAndShoot;
//    public static Pose flowerAndShootCtrl = redFlowerAndShootCtrl;u

    public static Pose blueDownStartPose = poseFactory.of(86.5, 132.5, 0);
    public static Pose blueGardenPose = poseFactory.of(129.5, 132.5, 0);

    //*TODO BLUE SOLO (2) POSES
    public static Pose blueFlowerAndShoot = poseFactory.of(87.65, 13.4179, 270);
    public static Pose blueFlowerAndShootCtrl = poseFactory.of(141.5 - 45.0655, 141.5 - 61.4432, 0);
    public static Pose blueFlowerAndShootCtrl2 = poseFactory.of(141.5 - 0, 141.5 - 94.659, 0);
//    private final Pose point2Control1 = poseFactory.of(45.0655, 61.4432, 0);
//    private final Pose point2Control2 = poseFactory.of(0, 94.659, 0);
    public static Pose blueSolo2Leave = poseFactory.of(128.2803, 24.9798, 270);

    //TODO RED SOLO 2 STEAL POSES
    public static Pose STEAL_FLOWER = poseFactory.of(89.4474, 12.2757, -90);
    public static Pose STEAL_FLOWER_CTRL = poseFactory.of(70.4972, 24.3503, 0);
    public static Pose STEAL_FLOWER_START = poseFactory.of(89.4474, 12.2757, 90);
    public static Pose STEAL_FLOWER_END = poseFactory.of(89.4474, 12.2757, -90);
    public static Pose OPP_FLOWER_START = poseFactory.of(89.4474, 12.2757, -90);
    public static Pose OPP_FLOWER_END = poseFactory.of(55.8037, 128.5187, 90);
    public static Pose OPP_FLOWER_CTRL1 = poseFactory.of(56.3977, 31.6485, 0);
    public static Pose OPP_FLOWER_CTRL_2 = poseFactory.of(82.6852, 107.8983, 0);

    public static class RedDownSolo2Path {
        public Path gardenIntake() {
            return Paths.line(redDownStartPose, redGardenPose).constant(redGardenPose);
        }

        public Path flowerAndShoot() {
            return Paths.curve(
                            redGardenPose, redFlowerAndShootCtrl, redFlowerAndShootCtrl2, redFlowerAndShoot
                    )
                    .linear(redGardenPose, redFlowerAndShoot);
        }

        public Path leave() {
            return Paths.line(redFlowerAndShoot, redSolo2Leave).constant(redSolo2Leave);
        }

        public Command autoRoutine(Follower follower, IntakeSS intakeSS, ShooterSS shooterSS) {
            return sequential(
                    //todo init
                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel(
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),
                    waitMs(Globals.shootRevDelayMs), //todo delay for the shooter to rev up to req velocity

                    //todo unlock and shoot
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    parallel(
                            follow(follower, gardenIntake()),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED))
                            //todo shooter locks during path and stays locked
                    ),

                    parallel(
                            follow(follower, flowerAndShoot()),
                            //todo shooter ss logic goes here to prepare for up side shoot
                            instant(() -> shooterSS.setAutoS3Up(true)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED))
                    ),
                    //todo flower ramp is deployed and shooter is unlocked for simultaneous shooting

                    //todo both unlocked in case scattered nectar is somehow intaked during path
                    waitMs(Globals.flowerAndShootDelayMs), //todo special delay for flowre intake and shoot, kept high cuz we have time

                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            follow(follower, leave())
                    ),
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.INIT)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.INIT))
                    )
                    //todo shut down and restore init after leave and park
            );
        }
    }

    public static class BlueDownSolo3STEALPath {
        PoseFactory poseFactory = PoseFactory.degrees();
        public final Pose startPose = poseFactory.of(55.6111, 8.7778, 90);
        private final Pose point2 = poseFactory.of(12.415, 51.9401, 220);
        private final Pose point2Control1 = poseFactory.of(19.1511, 96.2224, 0);
        private final Pose point7 = poseFactory.of(13.3988, 92.707, 97.6053);
        private final Pose point7Control1 = poseFactory.of(18.7138, 51.4447, 0);

        public Path adjacentFlowerShoot() {
            return Paths.line(point2, startPose).linear(point2, startPose);
        }

        public Path leave() {
            return Paths.curve(startPose, point7Control1, point7).tangent();
        }

        public Path adjacentFlower() {
            return Paths.curve(redFlowerAndShoot, point2Control1, point2).linear(redFlowerAndShoot, point2);
        }

        public Path STEAL_FLOWER() {
            return Paths.curve(startPose, STEAL_FLOWER_CTRL, STEAL_FLOWER).heading(Interpolator.piecewise().until(1, Interpolator.linear(STEAL_FLOWER_START, STEAL_FLOWER_END).reverse()));
        }

        public Path OPP_FLOWER() {
            return Paths.curve(OPP_FLOWER_START, OPP_FLOWER_CTRL1, OPP_FLOWER_CTRL_2, redFlowerAndShoot).linear(OPP_FLOWER_START, OPP_FLOWER_END);
        }

        public Path gardenIntake() {
            return Paths.line(startPose, redGardenPose).linear(90, 180);
        }

        public Path gardenShoot() {
            return Paths.line(redGardenPose, startPose).linear(180, 90);
        }

        public Command autoRoutine(Follower follower, IntakeSS intakeSS, ShooterSS shooterSS) {
            return sequential(
                    instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                    //todo init
                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel( //todo preload
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),
                    waitMs(Globals.shootRevDelayMs), //todo delay for the shooter to rev up to req velocity

                    //todo unlock and shoot
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    parallel(//todo steal flower
                            follow(follower, STEAL_FLOWER()),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED))
                    ),

                    instant(() -> shooterSS.setAutoS3Up(true)),
                    parallel( //todo opp flower
                            follow(follower, OPP_FLOWER()),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)), //todo flower up
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),

                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED))
                    ),
                    //todo flower ramp is deployed and shooter is unlocked for simultaneous shooting
                    waitMs(Globals.flowerAndShootDelayMs), //todo special delay for flowre intake and shoot, kept high cuz we have time

                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            follow(follower, adjacentFlower())
                    ),

                    instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)),
                    waitMs(Globals.flowerAndShootDelayMs),

                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),

                    follow(follower, adjacentFlowerShoot()),
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            follow(follower, gardenIntake())
                    ),

                    follow(follower, gardenShoot()),
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            follow(follower, leave())
                    ),

                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.INIT)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.INIT))
                    )
                    //todo shut down and restore init after leave and park
            );
        }
    }

    public static class BlueUpSolo2Path {
        public Path gardenIntake() {
            return Paths.line(blueDownStartPose, blueGardenPose).constant(blueGardenPose);
        }

        public Path flowerAndShoot() {
            return Paths.curve(
                            blueGardenPose, blueFlowerAndShootCtrl, blueFlowerAndShootCtrl2, blueFlowerAndShoot
                    )
                    .linear(blueGardenPose, blueFlowerAndShoot);
        }

        public Path leave() {
            return Paths.line(blueFlowerAndShoot, blueSolo2Leave).constant(blueSolo2Leave);
        }

        public Command autoRoutine(Follower follower, IntakeSS intakeSS, ShooterSS shooterSS) {
            return sequential(
                    //todo init
                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel(
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),
                    waitMs(Globals.shootRevDelayMs), //todo delay for the shooter to rev up to req velocity

                    //todo unlock and shoot
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    parallel(
                            follow(follower, gardenIntake()),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED))
                            //todo shooter locks during path and stays locked
                    ),

                    parallel(
                            follow(follower, flowerAndShoot()),
                            //todo shooter ss logic goes here to prepare for up side shoot
                            sequential(
                                    instant(() -> shooterSS.setAutoS3Up(true)),
                                    instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                                    instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                                    instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                            )
                    ),
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED))
                    ),
                    //todo flower ramp is deployed and shooter is unlocked for simultaneous shooting

                    //todo both unlocked in case scattered nectar is somehow intaked during path
                    waitMs(Globals.flowerAndShootDelayMs), //todo special delay for flowre intake and shoot, kept high cuz we have time

                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            follow(follower, leave())
                    ),
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.INIT)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.INIT))
                    )
                    //todo shut down and restore init after leave and park
            );
        }
    }

    public static class RedDownSolo3STEALPath {
        PoseFactory poseFactory = PoseFactory.degrees();
        public final Pose startPose = poseFactory.of(55.6111, 8.7778, 90);
        private final Pose point2 = poseFactory.of(12.415, 51.9401, 220);
        private final Pose point2Control1 = poseFactory.of(19.1511, 96.2224, 0);
        private final Pose point7 = poseFactory.of(13.3988, 92.707, 97.6053);
        private final Pose point7Control1 = poseFactory.of(18.7138, 51.4447, 0);

        public Path adjacentFlowerShoot() {
            return Paths.line(point2, startPose).linear(point2, startPose);
        }

        public Path leave() {
            return Paths.curve(startPose, point7Control1, point7).tangent();
        }

        public Path adjacentFlower() {
            return Paths.curve(redFlowerAndShoot, point2Control1, point2).linear(redFlowerAndShoot, point2);
        }

        public Path STEAL_FLOWER() {
            return Paths.curve(startPose, STEAL_FLOWER_CTRL, STEAL_FLOWER).heading(Interpolator.piecewise().until(1, Interpolator.linear(STEAL_FLOWER_START, STEAL_FLOWER_END).reverse()));
        }

        public Path OPP_FLOWER() {
            return Paths.curve(OPP_FLOWER_START, OPP_FLOWER_CTRL1, OPP_FLOWER_CTRL_2, redFlowerAndShoot).linear(OPP_FLOWER_START, OPP_FLOWER_END);
        }

        public Path gardenIntake() {
            return Paths.line(startPose, redGardenPose).linear(90, 180);
        }

        public Path gardenShoot() {
            return Paths.line(redGardenPose, startPose).linear(180, 90);
        }

        public Command autoRoutine(Follower follower, IntakeSS intakeSS, ShooterSS shooterSS) {
            return sequential(
                    instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                    //todo init
                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel( //todo preload
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),
                    waitMs(Globals.shootRevDelayMs), //todo delay for the shooter to rev up to req velocity

                    //todo unlock and shoot
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    parallel(//todo steal flower
                            follow(follower, STEAL_FLOWER()),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED))
                    ),

                    instant(() -> shooterSS.setAutoS3Up(true)),
                    parallel( //todo opp flower
                            follow(follower, OPP_FLOWER()),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)), //todo flower up
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),

                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED))
                    ),
                    //todo flower ramp is deployed and shooter is unlocked for simultaneous shooting
                    waitMs(Globals.flowerAndShootDelayMs), //todo special delay for flowre intake and shoot, kept high cuz we have time

                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            follow(follower, adjacentFlower())
                    ),

                    instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)),
                    waitMs(Globals.flowerAndShootDelayMs),

                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),

                    follow(follower, adjacentFlowerShoot()),
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            follow(follower, gardenIntake())
                    ),

                    follow(follower, gardenShoot()),
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            follow(follower, leave())
                    ),

                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.INIT)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.INIT))
                    )
                    //todo shut down and restore init after leave and park
            );
        }
    }

    public static class RedDownAlliance2Path {
        //TODO REVISIT THIS AFTER SCRIMMAGE, WILL FOCUS ON SCRIMMAGE SOLO AUTO FOR NOW
        public Path gardenIntake() {
            return Paths.line(redDownStartPose, redGardenPose).constant(redGardenPose);
        }

        public Path gardenShoot() {
            return Paths.line(redGardenPose, redDownStartPose).constant(redDownStartPose);
        }

        public Path flowerIntake() {
            return Paths.curve(redDownStartPose, redAllianceAdjFlowerCtrl, redAllianceAdjFlower).tangent();
        }

        public Path flowerShoot() {
            return Paths.line(redAllianceAdjFlower, redDownStartPose).linear(redAllianceAdjFlower, redDownStartPose);
        }
        public Path leave() {
            return Paths.curve(redDownStartPose, redDownLeaveCtrl, redDownLeave).tangent();
        }
    }

    public static class RedDownSolo3Path {

        private final PoseFactory poseFactory = PoseFactory.degrees();

        public final Pose startPose = poseFactory.of(63.0235, 9.4674, 180);
        private final Pose point1 = poseFactory.of(51.4989, 128.2668, 140);
        private final Pose point1Control1 = poseFactory.of(85.2139, 100.1134, 0);
        private final Pose point2Start = poseFactory.of(51.4989, 128.2668, 140);
        private final Pose point2 = poseFactory.of(12.415, 51.9401, 220);
        private final Pose point2Control1 = poseFactory.of(19.1511, 96.2224, 0);
        private final Pose point3Start = poseFactory.of(12.415, 51.9401, 220);
        private final Pose point3 = poseFactory.of(63.0235, 9.4674, 180);
        private final Pose point4 = poseFactory.of(9.8481, 7.3487, -177.7183);
        private final Pose point6 = poseFactory.of(63.0235, 9.4861, -177.7006);
        private final Pose point7 = poseFactory.of(13.3988, 92.707, 97.6053);
        private final Pose point7Control1 = poseFactory.of(18.7138, 51.4447, 0);

        public Path oppositeFlower() {
            return Paths.curve(startPose, point1Control1, point1).linear(startPose, point1);
        }

        public Path adjacentFlower() {
            return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
        }

        public Path adjacentFlowerShoot() {
            return Paths.line(point3Start, point3).linear(point3Start, point3);
        }

        public Path gardenIntake() {
            return Paths.line(point3, point4).tangent();
        }

        public Path gardenShoot() {
            return Paths.line(point4, point6).reverseTangent();
        }
        public Path leave() {
            return Paths.curve(startPose, point7Control1, point7).tangent();
        }
        public Command autoRoutine(Follower follower/*, IntakeSS intakeSS, ShooterSS shooterSS*/) {
            return sequential(
    //                //todo init
    //                parallel(
    //                        instant(()-> shooterSS.updateState(ShooterSS.ShooterState.AUTO)),
    //                        instant(()-> intakeSS.updateState(IntakeSS.IntakeState.ON)),
    //                        instant(()-> shooterSS.updateState(ShooterSS.HoodState.AUTO)),
    //                        instant(()-> shooterSS.updateState(ShooterSS.TurretState.AUTO))
    //                ),
    //                waitMs(Globals.shootRevDelayMs), //todo delay for the shooter to rev up to req velocity
    //
    //                //todo unlock and shoot
    //                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
    //                waitMs(Globals.shootDelayMs),
    //
    //                parallel(
    //                        follow(follower, oppositeFlower()),
    //                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED))
    //                        //todo shooter locked during path, unlocks at end
    //                ),
    //                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)), //todo flower ramp down for intake
    //                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
    //                waitMs(2000), //todo this is delay for intake and shoot
    //                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
    //                parallel(
    //                        follow(follower, adjacentFlower()),
    //                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED))
    //                        //todo shooter locked while intaking from flower
    //                ),
    //                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)),
    //                waitMs(1500), //todo this is delay for intake from flower needs tuning
    //                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
    //                follow(follower, adjacentFlowerShoot()),
    //                //todo unlock and shoot
    //                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
    //                waitMs(Globals.shootDelayMs),
    //                parallel(
    //                        follow(follower, gardenIntake()),
    //                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED))
    //                        //todo locked while intaking from garden
    //                ),
    //                follow(follower, gardenShoot()),
    //                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
    //                waitMs(Globals.shootDelayMs),
    //
    //                instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
    //                //todo locked while final intake while leaving^^

                    follow(follower, leave())
                    //todo leave n park
            );
        }
    }

    public static class BlueUpSolo3Path {

        private final PoseFactory poseFactory = PoseFactory.degrees();

        public final Pose startPose = poseFactory.of(78.4765, 132.0326, 0);
        private final Pose point1 = poseFactory.of(90.0011, 13.2332, 40);
        private final Pose point1Control1 = poseFactory.of(56.2861, 41.3866, 180);
        private final Pose point2Start = poseFactory.of(90.0011, 13.2332, 40);
        private final Pose point2 = poseFactory.of(129.085, 89.5599, -40);
        private final Pose point2Control1 = poseFactory.of(122.3489, 45.2776, 180);
        private final Pose point3Start = poseFactory.of(129.085, 89.5599, -40);
        private final Pose point3 = poseFactory.of(78.4765, 132.0326, 0);
        private final Pose point4 = poseFactory.of(131.6519, 134.1513, 357.7183);
        private final Pose point6 = poseFactory.of(78.4765, 132.0139, 357.7006);
        private final Pose point7 = poseFactory.of(128.1012, 48.793, 82.3947);
        private final Pose point7Control1 = poseFactory.of(122.7862, 90.0553, 0);

        public Path oppositeFlower() {
            return Paths.curve(startPose, point1Control1, point1).linear(startPose, point1);
        }

        public Path adjacentFlower() {
            return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
        }

        public Path adjacentFlowerShoot() {
            return Paths.line(point3Start, point3).linear(point3Start, point3);
        }

        public Path gardenIntake() {
            return Paths.line(point3, point4).tangent();
        }

        public Path gardenShoot() {
            return Paths.line(point4, point6).reverseTangent();
        }

        public Path leave() {
            return Paths.curve(startPose, point7Control1, point7).tangent();
        }

        public Command autoRoutine(Follower follower/*, IntakeSS intakeSS, ShooterSS shooterSS*/) {
            return sequential(
    //                //todo init
    //                parallel(
    //                        instant(()-> shooterSS.updateState(ShooterSS.ShooterState.AUTO)),
    //                        instant(()-> intakeSS.updateState(IntakeSS.IntakeState.ON)),
    //                        instant(()-> shooterSS.updateState(ShooterSS.HoodState.AUTO)),
    //                        instant(()-> shooterSS.updateState(ShooterSS.TurretState.AUTO))
    //                ),
    //                waitMs(Globals.shootRevDelayMs), //todo delay for the shooter to rev up to req velocity
    //
    //                //todo unlock and shoot
    //                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
    //                waitMs(Globals.shootDelayMs),
    //
    //                parallel(
    //                        follow(follower, oppositeFlower()),
    //                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED))
    //                        //todo shooter locked during path, unlocks at end
    //                ),
    //                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)), //todo flower ramp down for intake
    //                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
    //                waitMs(2000), //todo this is delay for intake and shoot
    //                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
    //                parallel(
    //                        follow(follower, adjacentFlower()),
    //                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED))
    //                        //todo shooter locked while intaking from flower
    //                ),
    //                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)),
    //                waitMs(1500), //todo this is delay for intake from flower needs tuning
    //                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
    //                follow(follower, adjacentFlowerShoot()),
    //                //todo unlock and shoot
    //                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
    //                waitMs(Globals.shootDelayMs),
    //                parallel(
    //                        follow(follower, gardenIntake()),
    //                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED))
    //                        //todo locked while intaking from garden
    //                ),
    //                follow(follower, gardenShoot()),
    //                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
    //                waitMs(Globals.shootDelayMs),
    //
    //                instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
    //                //todo locked while final intake while leaving^^

                    follow(follower, leave())
                    //todo leave n park
            );
        }
    }
}