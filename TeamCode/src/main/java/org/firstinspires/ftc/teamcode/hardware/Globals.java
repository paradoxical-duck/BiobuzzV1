package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class Globals {
    public static double intakeOn = 1;
    public static double intakeOff = 1;
    public static double intakeRev = -1;
    public static double shooterOn = 1000; //todo tune
    public static double shooterInit = 0;
    public static double turretInit = 0.5;
    public static double hoodShootPos = 0.5; //todo tune
    public static double hoodInit = 0.5;
    public static double turretShootPos = 0.5;
    //todo poses for odo turret tracking
    public static Pose redHiveUpSide;
    public static Pose redHiveDownSide;
    public static Pose blueHiveUpSide;
    public static Pose blueHiveDownSide;
    //todo shooter PIDF
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
}
