package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

// extra imports
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class RobotHardware {
    public List<LynxModule> allHubs;
    //TODO =====  INTAKE=====
    public DcMotorEx intake1;
    public DcMotorEx intake2;
    public ServoImplEx intakeDeploy;

    //TODO =====  SHOOTER - Pollen  =====
    public DcMotorEx shooter1;  //Pollen
    public ServoImplEx turret1;
    public ServoImplEx hood1;

    //TODO =====  SHOOTER - Nectar  =====
    public DcMotorEx shooter2;  //Nectar
    public ServoImplEx turret2;
    public ServoImplEx hood2;

    //TODO =====  FEEDER LOCKS  =====
    public ServoImplEx feedLock1; //Pollen
    public ServoImplEx feedLock2; //Nectar


    //TODO =====  BEAM BREAKERS  =====
    public DigitalChannel frontBeam;
    public DigitalChannel midBeam;
    public DigitalChannel pollenBeam;
    public DigitalChannel nectarBeam;

    //TODO =====  COLOR SENSOR  =====
    public RevColorSensorV3 colorSensor;

    //TODO ======= ARDUCAM =========
//    public AprilTagProcessor arducam;
    public VisionPortal visionPortal;
    //TODO =====  ALLIANCE  =====
    public enum Alliance { RED, BLUE }
    public static Alliance alliance = Alliance.RED; // static so auto's choice carries into teleop

    public static boolean isRed() { return alliance == Alliance.RED; }
    public static void toggleAlliance() { alliance = isRed() ? Alliance.BLUE : Alliance.RED; }
    public static Alliance getAlliance() {
        return alliance;
    }
    public static void setAlliance(Alliance newAlliance) {
        alliance = newAlliance;
    }

    //TODO =====  GAMEPADS (for toggles)  =====
    public final Gamepad currentGamepad1 = new Gamepad();
    public final Gamepad currentGamepad2 = new Gamepad();
    public final Gamepad previousGamepad1 = new Gamepad();
    public final Gamepad previousGamepad2 = new Gamepad();

    //TODO =====  VOLTAGE + LOOP TIME  =====
    public VoltageSensor voltageSensor;
    public double voltage = 12;
    public double loopTimeMs = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    public boolean enabled = false;
    private HardwareMap hardwareMap ;

    public static RobotHardware instance = null;
    public static Pose currentPose = new Pose(0,0,0);

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        //TODO =====  INTAKE=====
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeDeploy = hardwareMap.get(ServoImplEx.class, "intakeDeploy");

        //TODO =====  SHOOTER - Pollen  =====
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        turret1 = hardwareMap.get(ServoImplEx.class, "turret1");
        hood1 = hardwareMap.get(ServoImplEx.class, "hood1");
//        shooter1.setVelocityPIDFCoefficients(P, I, D, F);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //TODO =====  SHOOTER - Nectar  =====
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        turret2 = hardwareMap.get(ServoImplEx.class, "turret2");
        hood2 = hardwareMap.get(ServoImplEx.class, "hood2");
//        shooter1.setVelocityPIDFCoefficients(P, I, D, F);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyShooterPIDF();

        //TODO =====  FEEDER LOCKS  =====
        feedLock1 = hardwareMap.get(ServoImplEx.class, "feedLock1");  //Pollen
        feedLock2 = hardwareMap.get(ServoImplEx.class, "feedLock2");  //Nectar


        //TODO =====  BEAM BREAKERS  =====
        pollenBeam = hardwareMap.get(DigitalChannel.class, "pollenBeam");
        nectarBeam = hardwareMap.get(DigitalChannel.class, "nectarBeam");
        midBeam = hardwareMap.get(DigitalChannel.class, "midBeam");
        frontBeam = hardwareMap.get(DigitalChannel.class, "frontBeam");

        //TODO =====  COLOR SENSOR  =====
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color sensor");

        //TODO ===== ARDUCAM =====
//        arducam = AprilTagProcessor.easyCreateWithDefaults();
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "arducam"))
//                .addProcessor(arducam)
//                .build();

        //TODO ==== VOLTAGE SENSOR ========
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    public void clearBulkCache() {
        if (allHubs == null) return;

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    /** Updates loop time and voltage. Call robot.clearBulkCache() separately. */
    public void update() {
        loopTimeMs = loopTimer.milliseconds();
        loopTimer.reset();
        voltage = voltageSensor.getVoltage();
    }

    /** Call once per loop, before reading buttons. */
    public void readGamepads(Gamepad gamepad1, Gamepad gamepad2) {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }

    /** True only on the loop the button goes down. */
    public static boolean pressed(boolean now, boolean last) { return now && !last; }

    /** Beam breakers read false when something is blocking them. */
    public static boolean blocked(DigitalChannel beam) { return !beam.getState(); }

    /** Sets heading to 0 and keeps x/y. Use for field-centric drive. */
    public static void resetYaw(Follower follower) {
        Pose p = follower.pose();
        follower.setPose(new Pose(p.x(), p.y(), 0));
    }

    /** Re-applies the PIDF values from Globals, so Panels edits take effect. */
    public void applyShooterPIDF() {
        if (Globals.kP_Pollen != 0 || Globals.kI_Pollen != 0 || Globals.kD_Pollen != 0 || Globals.kF_Pollen != 0) {
            shooter1.setVelocityPIDFCoefficients(Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
        }
        if (Globals.kP_Nectar != 0 || Globals.kI_Nectar != 0 || Globals.kD_Nectar != 0 || Globals.kF_Nectar != 0) {
            shooter2.setVelocityPIDFCoefficients(Globals.kP_Nectar, Globals.kI_Nectar, Globals.kD_Nectar, Globals.kF_Nectar);
        }
    }

    /** Stops every motor. Use at the end of an auto or in an emergency. */
    public void stopAll() {
        intake1.setPower(0);
        intake2.setPower(0);
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
    }

    /** Scales a motor power so it acts the same at low battery. */
    public double voltageComp(double power) { return power * 12.0 / voltage; }

    public double amps(DcMotorEx motor) { return motor.getCurrent(CurrentUnit.AMPS); }
    public boolean isEmpty() {
        return frontBeam.getState() &&
                pollenBeam.getState() &&
                nectarBeam.getState() &&
                midBeam.getState();
    }

}