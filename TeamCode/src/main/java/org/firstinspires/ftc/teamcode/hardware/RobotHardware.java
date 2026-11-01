package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Configurable
public class RobotHardware {
    public DcMotorEx intake1;
    public DcMotorEx intake2;
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public ServoImplEx turret;
    public ServoImplEx hood;
    public ServoImplEx flowerRamp;
    public ServoImplEx lock1;
    public ServoImplEx lock2;
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
    // todo beambreakers
    public DigitalChannel frontBeam;
    public DigitalChannel midBeam;
    public DigitalChannel pollenBeam;
    public DigitalChannel nectarBeam;
    public static boolean enabled = false;
    public static RobotHardware instance;
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }
    public void init(HardwareMap hardwareMap) {
       intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
       intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

       intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
       shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

       shooter1.setVelocityPIDFCoefficients(P, I, D, F);
       shooter2.setVelocityPIDFCoefficients(P, I, D, F);

       turret = hardwareMap.get(ServoImplEx.class, "turret");
       //TODO TURRET MAY BE 2 SERVOSSSSSSSSS
       hood = hardwareMap.get(ServoImplEx.class, "hood");

       flowerRamp = hardwareMap.get(ServoImplEx.class, "flower ramp");
       lock1 = hardwareMap.get(ServoImplEx.class, "lock1");
       lock2 = hardwareMap.get(ServoImplEx.class, "lock2");
       //todo beambreakers
        pollenBeam = hardwareMap.get(DigitalChannel.class, "pollenBeam");
        nectarBeam = hardwareMap.get(DigitalChannel.class, "nectarBeam");
        midBeam = hardwareMap.get(DigitalChannel.class, "midBeam");
        frontBeam = hardwareMap.get(DigitalChannel.class, "frontBeam");
    }
}
