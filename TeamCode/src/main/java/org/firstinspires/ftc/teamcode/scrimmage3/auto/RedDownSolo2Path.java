package org.firstinspires.ftc.teamcode.scrimmage3.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

public class RedDownSolo2Path {
    public Path gardenIntake() {
        return Paths.line(AutoConstants.redDownStartPose, AutoConstants.redGardenPose).constant(AutoConstants.redGardenPose);
    }

    public Path flowerAndShoot() {
        return Paths.curve(
                AutoConstants.redGardenPose, AutoConstants.flowerAndShootCtrl, AutoConstants.flowerAndShoot
        )
                .linear(AutoConstants.redGardenPose, AutoConstants.flowerAndShoot);
    }

    public Path leave() {
        return Paths.line(AutoConstants.flowerAndShoot, AutoConstants.redSolo2Leave).constant(AutoConstants.redSolo2Leave);
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
                instant(()-> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                waitMs(Globals.shootDelayMs),

                parallel(
                        follow(follower, gardenIntake()),
                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED))
                        //todo shooter locks during path and stays locked
                ),

                parallel(
                        follow(follower, flowerAndShoot())/*,*/
                        //todo shooter ss logic goes here to prepare for up side shoot
                ),
                parallel(
                        instant(()-> shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED)),
                        instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED))
                        ),
                ///todo flower ramp is deployed and shooter is unlocked for simultaneous shooting

                //todo both unlocked in case scattered nectar is somehow intaked during path
                waitMs(Globals.flowerAndShootDelayMs), //todo special delay for flowre intake and shoot, kept high cuz we have time

                parallel(
                        instant(()-> shooterSS.updateState(ShooterSS.LockState.LOCKED)),
                        instant(()-> intakeSS.updateState(IntakeSS.FlowerRampState.UP)),
                        follow(follower, leave())
                ),
                parallel(
                        instant(()-> intakeSS.updateState(IntakeSS.IntakeState.INIT)),
                        instant(()-> shooterSS.updateState(ShooterSS.ShooterState.INIT))
                )
                //todo shut down and restore init after leave and park
        );
    }
}
