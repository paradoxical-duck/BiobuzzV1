
package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public class Globals {
    public static double intakeOn = 1;
    public static double intakeOff = 0;
    public static double intakeRev = -1;
    public static double shooterOn = 1000; //todo tune
    public static double shooterInit = 0;
    public static double turretInit = 0.5;
    public static double hoodShootPos = 0.5; //todo tune
    public static double hoodInit = 0.5;
    public static double turretShootPos = 0.5;
    //todo poses for odo turret tracking
    public static Pose redHiveDownSide = new Pose(57.75, 52.5292);
    public static Pose redHiveUpSide = new Pose(57.75, 85.5877);
    public static Pose blueHiveDownSide = new Pose(84.19, 49.6123);
    public static Pose blueHiveUpSide = new Pose(84.19, 87.1434);
    public static double turretOffset = 0.0; //TODO MAY BE REQUIRED FOR DOUBBLE TURRET WITH 2 SERVOS
    //todo shooter PIDF
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    //todo flower ramp poses
    public static double flowerRampUp = 0.5;
    public static double flowerRampDeployed = 0.5;
    public static double flowerRampInit = 0.5;
    //todo TUNE^^^^^^^^^^

    //todo shooter lock poses
    public static double locked = 0.5;
    public static double unlocked = 0.5;
    //todo tune
    public static double shootDelayMs = 500; //todo figure out actual delay
    public static double shootRevDelayMs = 250; //todo figure out
    //todo auto shooter values todo tune
    public static double autoShoot = 1000;
    public static double autoTurret = 0.5;
    public static double autoHood = 0.5;

    public static double colorMaxDistanceCm = 5.0;
    public static double colorMinSaturation = 0.30;
    public static double pollenHue = 52;
    public static double pollenHueTolerance = 18;
    public static double redNectarHue = 0;
    public static double redNectarHueTolerance = 15;
    public static double blueNectarHue = 220;
    public static double blueNectarHueTolerance = 20;



    public static double kP_Pollen = 0;
    public static double kI_Pollen = 0;
    public static double kD_Pollen = 0;
    public static double kF_Pollen = 0;

    public static double kP_Nectar = 0;
    public static double kI_Nectar = 0;
    public static double kD_Nectar = 0;
    public static double kF_Nectar = 0;

}
