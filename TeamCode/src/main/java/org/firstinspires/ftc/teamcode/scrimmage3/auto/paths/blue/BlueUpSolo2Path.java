package org.firstinspires.ftc.teamcode.scrimmage3.auto.paths.blue;

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

public class BlueUpSolo2Path {
    public Path gardenIntake() {
        return Paths.line(AutoConstants.blueDownStartPose, AutoConstants.blueGardenPose).constant(AutoConstants.blueGardenPose);
    }

    public Path flowerAndShoot() {
        return Paths.curve(
                        AutoConstants.blueGardenPose, AutoConstants.blueFlowerAndShootCtrl, AutoConstants.blueFlowerAndShoot
                )
                .linear(AutoConstants.blueGardenPose, AutoConstants.blueFlowerAndShoot);
    }

    public Path leave() {
        return Paths.line(AutoConstants.blueFlowerAndShoot, AutoConstants.blueSolo2Leave).constant(AutoConstants.blueSolo2Leave);
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