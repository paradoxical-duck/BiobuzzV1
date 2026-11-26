package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Config       // live-editable in FTC Dashboard -> Config -> Globals (click SAVE)
@Configurable // also live-editable in Panels -> Configurables
public class Globals {

    //todo intake
    public static double intakeOn = 1;
    public static double intakeOff = 0;
    public static double intakeRev = -1;

    //todo shooter ss
    public static double shooterInit = 0;

    public static double turretInit = 0.5; //todo Nectar (turret2) init, Pollen uses turret1Init
    public static double hoodInit = 0.33;

    //todo turret1 (Pollen) - tuned with ShooterPollenTuner
    public static double turret1Init = 0.62;       // faces straight front
    public static double turret1RightLimit = 0.48; // lower value = turns right (seen from back), mech limit
    public static double turret1LeftLimit = 0.83;  // higher value = turns left, limit for now

    //todo hood1 (Pollen) - tuned with ShooterPollenTuner
    public static double hood1Up = 0.33;   // max up
    public static double hood1Down = 0.51; // max down

    //todo shooter ss pollen
    public static double shooterOnPollen = 1650; // ticks/s, tuned

    public static double turretShootPosPollen = 0.5;
    public static double hoodShootPosPollen = 0.4; // tuned

    //todo shooter ss nectar
    public static double shooterOnNectar = 1000; //todo velocity in ticks/s
    public static double turretShootPosNectar = 0.5;
    public static double hoodShootPosNectar = 0.33;

    //todo poses for odo turret tracking
    public static Pose redHiveDownSide = new Pose(57.75, 52.5292);
    public static Pose redHiveUpSide = new Pose(57.75, 85.5877);
    public static Pose blueHiveDownSide = new Pose(84.19, 49.6123);
    public static Pose blueHiveUpSide = new Pose(84.19, 87.1434);

    //todo turret offsets for pollen and nectar
    public static double turretOffsetPollen = 0.0;
    public static double turretOffsetNectar = 0.0;

    //todo flower deploy poses
    public static double intakeDeployUp = 0.67;
    public static double intakeDeployDown = 0.28;
    public static double intakeDeployInit = 0.67;
    //todo TUNE^^^^^^^^^^

    //todo shooter lock poses POLLEN
    public static double feedLock1Open = 0.75;  // pass
    public static double feedLock1Close = 0.54; // block, also init
    //todo shooter lock poses NECTAR
    public static double feedLock2Open = 0.5;
    public static double feedLock2Close = 0.5;
    //todo tune
    public static double shootDelayMs = 500; //todo figure out actual delay
    public static double shootRevDelayMs = 250; //todo figure out
    public static double flowerAndShootDelayMs = 4000;

    //todo auto values pollen
    public static double autoShootPollen = 1000; //todo ticks/s
    public static double autoTurretPollen = 0.5;
    public static double autoHoodPollen = 0.33;

    //todo auto values nectar
    public static double autoShootNectar = 1000; //todo ticks/s
    public static double autoTurretNectar = 0.5;
    public static double autoHoodNectar = 0.33;

    //todo color sensor
    public static double colorMaxDistanceCm = 5.0;
    public static double colorMinSaturation = 0.30;
    public static double pollenHue = 52;
    public static double pollenHueTolerance = 18;
    public static double redNectarHue = 0;
    public static double redNectarHueTolerance = 15;
    public static double blueNectarHue = 220;
    public static double blueNectarHueTolerance = 20;

    //todo shooter pidf values pollen
    public static double kP_Pollen = 30; // tuned
    public static double kI_Pollen = 0;
    public static double kD_Pollen = 0;
    public static double kF_Pollen = 12.5; // tuned

    //todo shooter pidf values nectar
    public static double kP_Nectar = 0;
    public static double kI_Nectar = 0;
    public static double kD_Nectar = 0;
    public static double kF_Nectar = 0;

    //TODO AUTO VALUES SCRIMMAGE 3
    public static double pollenShootDown = 1000; //todo ticks/s
    public static double pollenShootUp = 1000; //todo ticks/s
    public static double pollenTurretDown = 0.5;
    public static double pollenTurretUp = 0.5;
    public static double pollenHoodUp = 0.33;
    public static double pollenHoodDown = 0.33;
    //todo these are all shoot values for either up side or down side, will mostly remain similar btwn up and down side except turret
    //todo note: these values should work across blue and red alliance cuz poses are mirrored
}