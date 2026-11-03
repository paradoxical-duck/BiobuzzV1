package org.firstinspires.ftc.teamcode.auto.paths;

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

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

public class BlueUpSolo3Path {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(78.4765, 132.0326, 90);
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
    private final Pose point7Control1 = poseFactory.of(122.7862, 90.0553, 180);

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
        return Paths.curve(point6, point7Control1, point7).tangent();
    }

    public Command autoRoutine(Follower follower, IntakeSS intakeSS, ShooterSS shooterSS) {
        return sequential(
                //todo init
                parallel(
                        instant(()-> shooterSS.updateState(ShooterSS.ShooterState.AUTO)),
                        instant(()-> intakeSS.updateState(IntakeSS.IntakeState.ON)),
                        instant(()-> shooterSS.updateState(ShooterSS.HoodState.AUTO)),
                        instant(()-> shooterSS.updateState(ShooterSS.TurretState.AUTO))
                ),
                waitMs(Globals.shootRevDelayMs), //todo delay for the shooter to rev up to req velocity

                //todo unlock and shoot
                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
                waitMs(Globals.shootDelayMs),

                parallel(
                        follow(follower, oppositeFlower()),
                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED))
                        //todo shooter locked during path, unlocks at end
                ),
                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)), //todo flower ramp down for intake
                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
                waitMs(2000), //todo this is delay for intake and shoot
                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                parallel(
                        follow(follower, adjacentFlower()),
                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED))
                        //todo shooter locked while intaking from flower
                ),
                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED)),
                waitMs(1500), //todo this is delay for intake from flower needs tuning
                instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                follow(follower, adjacentFlowerShoot()),
                //todo unlock and shoot
                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
                waitMs(Globals.shootDelayMs),
                parallel(
                        follow(follower, gardenIntake()),
                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED))
                        //todo locked while intaking from garden
                ),
                follow(follower, gardenShoot()),
                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
                waitMs(Globals.shootDelayMs),

                instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                //todo locked while final intake while leaving^^

                follow(follower, leave())
                //todo leave n park
        );
    }
}