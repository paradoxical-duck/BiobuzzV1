
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
    public static final PoseFactory poseFactory = PoseFactory.degrees();
    private static final double FIELD_WIDTH = 141.5;

    public static Pose mirror(Pose pose) {
        return poseFactory.of(FIELD_WIDTH - pose.x(), pose.y(), (Math.toDegrees(pose.heading()) + 180) % 360);
    }

    // Red down start, garden and alliance flower
    public static final Pose redDownStartPose = poseFactory.of(55, 9, 180);
    public static final Pose redGardenPose = poseFactory.of(12, 9, 180);
    public static final Pose redAllianceAdjFlower = poseFactory.of(10.7389, 37.7114, 111.3137);
    public static final Pose redAllianceAdjFlowerCtrl = poseFactory.of(19.7874, 14.5196, 0);
    public static final Pose redDownLeave = poseFactory.of(13.3988, 92.707, 97.6053);
    public static final Pose redDownLeaveCtrl = poseFactory.of(18.7138, 51.4447, 0);

    // Red solo 2
    public static final Pose redFlowerAndShoot = poseFactory.of(53.85, 128.0821, 90);
    public static final Pose redFlowerAndShootCtrl = poseFactory.of(45.0655, 61.4432, 0);
    public static final Pose redFlowerAndShootCtrl2 = poseFactory.of(0, 94.659, 0);
    public static final Pose redSolo2Leave = poseFactory.of(13.2197, 116.5202, 90);

    // Blue solo 2
    public static final Pose blueDownStartPose = poseFactory.of(86.5, 132.5, 0);
    public static final Pose blueGardenPose = poseFactory.of(129.5, 132.5, 0);
    public static final Pose blueFlowerAndShoot = poseFactory.of(87.65, 13.4179, 270);
    public static final Pose blueFlowerAndShootCtrl = poseFactory.of(96.4345, 80.0568, 0);
    public static final Pose blueFlowerAndShootCtrl2 = poseFactory.of(141.5, 46.841, 0);
    public static final Pose blueSolo2Leave = poseFactory.of(128.2803, 24.9798, 270);

    // Red STEAL route
    public static final Pose STEAL_FLOWER = poseFactory.of(89.4474, 12.2757, -90);
    public static final Pose STEAL_FLOWER_CTRL = poseFactory.of(70.4972, 24.3503, 0);
    public static final Pose STEAL_FLOWER_START = poseFactory.of(89.4474, 12.2757, 90);
    public static final Pose STEAL_FLOWER_END = poseFactory.of(89.4474, 12.2757, -90);
    public static final Pose OPP_FLOWER_START = poseFactory.of(89.4474, 12.2757, -90);
    public static final Pose OPP_FLOWER_END = poseFactory.of(55.8037, 128.5187, 90);
    public static final Pose OPP_FLOWER_CTRL1 = poseFactory.of(56.3977, 31.6485, 0);
    public static final Pose OPP_FLOWER_CTRL_2 = poseFactory.of(82.6852, 107.8983, 0);

    // Blue STEAL route mirrors X and rotates heading by 180 degrees
    public static final Pose BLUE_STEAL_FLOWER = mirror(STEAL_FLOWER);
    public static final Pose BLUE_STEAL_FLOWER_CTRL = mirror(STEAL_FLOWER_CTRL);
    public static final Pose BLUE_STEAL_FLOWER_START = mirror(STEAL_FLOWER_START);
    public static final Pose BLUE_STEAL_FLOWER_END = mirror(STEAL_FLOWER_END);
    public static final Pose BLUE_OPP_FLOWER_START = mirror(OPP_FLOWER_START);
    public static final Pose BLUE_OPP_FLOWER_END = mirror(OPP_FLOWER_END);
    public static final Pose BLUE_OPP_FLOWER_CTRL1 = mirror(OPP_FLOWER_CTRL1);
    public static final Pose BLUE_OPP_FLOWER_CTRL_2 = mirror(OPP_FLOWER_CTRL_2);

    // Shared red solo 3 STEAL geometry
    public static final Pose redStealStartPose = poseFactory.of(55.6111, 8.7778, 90);
    public static final Pose redStealAdjacentFlower = poseFactory.of(12.415, 51.9401, 220);
    public static final Pose redStealAdjacentFlowerCtrl = poseFactory.of(19.1511, 96.2224, 0);
    public static final Pose redStealLeave = poseFactory.of(13.3988, 92.707, 97.6053);
    public static final Pose redStealLeaveCtrl = poseFactory.of(18.7138, 51.4447, 0);

    // Mirrored blue solo 3 STEAL geometry
    public static final Pose blueStealStartPose = mirror(redStealStartPose);
    public static final Pose blueStealAdjacentFlower = mirror(redStealAdjacentFlower);
    public static final Pose blueStealAdjacentFlowerCtrl = mirror(redStealAdjacentFlowerCtrl);
    public static final Pose blueStealLeave = mirror(redStealLeave);
    public static final Pose blueStealLeaveCtrl = mirror(redStealLeaveCtrl);
    public static final Pose blueStealFlowerAndShoot = mirror(redFlowerAndShoot);
    public static final Pose blueStealGardenPose = mirror(redGardenPose);

    public static class RedDownSolo2Path {
        public Path gardenIntake() {
            return Paths.line(redDownStartPose, redGardenPose).constant(redGardenPose);
        }

        public Path flowerAndShoot() {
            return Paths.curve(redGardenPose, redFlowerAndShootCtrl, redFlowerAndShootCtrl2, redFlowerAndShoot).linear(redGardenPose, redFlowerAndShoot);
        }

        public Path leave() {
            return Paths.line(redFlowerAndShoot, redSolo2Leave).constant(redSolo2Leave);
        }

        public Command autoRoutine(Follower follower, IntakeSS intakeSS, ShooterSS shooterSS) {
            return sequential(
                    // Configure the preload shot
                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel(
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),
                    waitMs(Globals.shootRevDelayMs),

                    // Fire the preload
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    parallel(
                            follow(follower, gardenIntake()),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED))
                    ),

                    // Configure the far-side shot during travel
                    parallel(
                            follow(follower, flowerAndShoot()),
                            instant(() -> shooterSS.setAutoS3Up(true)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),

                    // Intake from the flower and shoot
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED))
                    ),
                    waitMs(Globals.flowerAndShootDelayMs),

                    // Leave and park
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            follow(follower, leave())
                    ),
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.INIT)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.INIT))
                    )
            );
        }
    }

    public static class BlueUpSolo2Path {
        public Path gardenIntake() {
            return Paths.line(blueDownStartPose, blueGardenPose).constant(blueGardenPose);
        }

        public Path flowerAndShoot() {
            return Paths.curve(blueGardenPose, blueFlowerAndShootCtrl, blueFlowerAndShootCtrl2, blueFlowerAndShoot).linear(blueGardenPose, blueFlowerAndShoot);
        }

        public Path leave() {
            return Paths.line(blueFlowerAndShoot, blueSolo2Leave).constant(blueSolo2Leave);
        }

        public Command autoRoutine(Follower follower, IntakeSS intakeSS, ShooterSS shooterSS) {
            return sequential(
                    // Configure the preload shot
                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel(
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),
                    waitMs(Globals.shootRevDelayMs),

                    // Fire the preload
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    parallel(
                            follow(follower, gardenIntake()),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED))
                    ),

                    // Configure the far-side shot during travel
                    parallel(
                            follow(follower, flowerAndShoot()),
                            sequential(
                                    instant(() -> shooterSS.setAutoS3Up(true)),
                                    instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                                    instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                                    instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                            )
                    ),

                    // Intake from the flower and shoot
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED))
                    ),
                    waitMs(Globals.flowerAndShootDelayMs),

                    // Leave and park
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            follow(follower, leave())
                    ),
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.INIT)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.INIT))
                    )
            );
        }
    }

    public static class RedDownSolo3STEALPath {
        public final Pose startPose = redStealStartPose;
        private final Pose point2 = redStealAdjacentFlower;
        private final Pose point2Control1 = redStealAdjacentFlowerCtrl;
        private final Pose point7 = redStealLeave;
        private final Pose point7Control1 = redStealLeaveCtrl;

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
                    // Configure and fire the preload
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel(
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),
                    waitMs(Globals.shootRevDelayMs),
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    // Steal the flower
                    parallel(
                            follow(follower, STEAL_FLOWER()),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED))
                    ),

                    // Move to the opposite flower
                    instant(() -> shooterSS.setAutoS3Up(true)),
                    parallel(
                            follow(follower, OPP_FLOWER()),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),

                    // Intake and shoot at the opposite flower
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED))
                    ),
                    waitMs(Globals.flowerAndShootDelayMs),

                    // Move to the adjacent flower
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            follow(follower, adjacentFlower())
                    ),

                    // Intake from the adjacent flower
                    instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)),
                    waitMs(Globals.flowerAndShootDelayMs),

                    // Prepare the down-side shot
                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),

                    // Shoot the collected pollen
                    follow(follower, adjacentFlowerShoot()),
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    // Collect from the garden
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            follow(follower, gardenIntake())
                    ),

                    // Return and shoot
                    follow(follower, gardenShoot()),
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    // Leave and park
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            follow(follower, leave())
                    ),
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.INIT)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.INIT))
                    )
            );
        }
    }

    public static class BlueDownSolo3STEALPath {
        public final Pose startPose = blueStealStartPose;
        private final Pose point2 = blueStealAdjacentFlower;
        private final Pose point2Control1 = blueStealAdjacentFlowerCtrl;
        private final Pose point7 = blueStealLeave;
        private final Pose point7Control1 = blueStealLeaveCtrl;

        public Path adjacentFlowerShoot() {
            return Paths.line(point2, startPose).linear(point2, startPose);
        }

        public Path leave() {
            return Paths.curve(startPose, point7Control1, point7).tangent();
        }

        public Path adjacentFlower() {
            return Paths.curve(blueStealFlowerAndShoot, point2Control1, point2).linear(blueStealFlowerAndShoot, point2);
        }

        public Path BLUE_STEAL_FLOWER() {
            return Paths.curve(startPose, BLUE_STEAL_FLOWER_CTRL, BLUE_STEAL_FLOWER).heading(Interpolator.piecewise().until(1, Interpolator.linear(BLUE_STEAL_FLOWER_START, BLUE_STEAL_FLOWER_END).reverse()));
        }

        public Path OPP_FLOWER() {
            return Paths.curve(BLUE_OPP_FLOWER_START, BLUE_OPP_FLOWER_CTRL1, BLUE_OPP_FLOWER_CTRL_2, blueStealFlowerAndShoot).linear(BLUE_OPP_FLOWER_START, BLUE_OPP_FLOWER_END);
        }

        public Path gardenIntake() {
            return Paths.line(startPose, blueStealGardenPose).linear(270, 360);
        }

        public Path gardenShoot() {
            return Paths.line(blueStealGardenPose, startPose).linear(360, 270);
        }

        public Command autoRoutine(Follower follower, IntakeSS intakeSS, ShooterSS shooterSS) {
            return sequential(
                    // Configure and fire the preload
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel(
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),
                    waitMs(Globals.shootRevDelayMs),
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    // Steal the flower
                    parallel(
                            follow(follower, BLUE_STEAL_FLOWER()),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED))
                    ),

                    // Move to the opposite flower
                    instant(() -> shooterSS.setAutoS3Up(true)),
                    parallel(
                            follow(follower, OPP_FLOWER()),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),

                    // Intake and shoot at the opposite flower
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)),
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED))
                    ),
                    waitMs(Globals.flowerAndShootDelayMs),

                    // Move to the adjacent flower
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            follow(follower, adjacentFlower())
                    ),

                    // Intake from the adjacent flower
                    instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)),
                    waitMs(Globals.flowerAndShootDelayMs),

                    // Prepare the down-side shot
                    instant(() -> shooterSS.setAutoS3Up(false)),
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.HoodState.AUTO_S3)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.TurretState.AUTO_S3))
                    ),

                    // Shoot the collected pollen
                    follow(follower, adjacentFlowerShoot()),
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    // Collect from the garden
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            follow(follower, gardenIntake())
                    ),

                    // Return and shoot
                    follow(follower, gardenShoot()),
                    instant(() -> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                    waitMs(Globals.shootDelayMs),

                    // Leave and park
                    parallel(
                            instant(() -> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                            follow(follower, leave())
                    ),
                    parallel(
                            instant(() -> intakeSS.updateState(IntakeSS.IntakeState.INIT)),
                            instant(() -> shooterSS.updatePollenState(ShooterSS.ShooterState.INIT))
                    )
            );
        }
    }

    public static class RedDownAlliance2Path {
        // Path definitions only; routine not implemented
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

        public Command autoRoutine(Follower follower) {
            return sequential(follow(follower, leave()));
        }
    }

    public static class BlueUpSolo3Path {
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

        public Command autoRoutine(Follower follower) {
            return sequential(follow(follower, leave()));
        }
    }
}
