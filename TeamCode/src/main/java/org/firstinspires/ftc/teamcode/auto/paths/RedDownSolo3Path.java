
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
import org.firstinspires.ftc.teamcode.subsystems.FlowerRampSS;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

public class RedDownSolo3Path {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(63.0235, 9.4674, 90);
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
        return Paths.curve(point6, point7Control1, point7).tangent();
    }
    public Command autoRoutine(Follower follower, IntakeSS intakeSS, FlowerRampSS flowerRampSS, ShooterSS shooterSS) {
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
                instant(()-> flowerRampSS.updateState(FlowerRampSS.FlowerRampState.DEPLOYED)), //todo flower ramp down for intake
                instant(()-> shooterSS.updateState(ShooterSS.LockState.UNLOCKED)),
                waitMs(2000), //todo this is delay for intake and shoot
                instant(()-> flowerRampSS.updateState(FlowerRampSS.FlowerRampState.UP)),
                parallel(
                        follow(follower, adjacentFlower()),
                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED))
                        //todo shooter locked while intaking from flower
                ),
                instant(()-> flowerRampSS.updateState(FlowerRampSS.FlowerRampState.DEPLOYED)),
                waitMs(1500), //todo this is delay for intake from flower needs tuning
                instant(()-> flowerRampSS.updateState(FlowerRampSS.FlowerRampState.UP)),
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
