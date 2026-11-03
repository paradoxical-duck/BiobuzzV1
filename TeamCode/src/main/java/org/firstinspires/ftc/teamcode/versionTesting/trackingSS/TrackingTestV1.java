package org.firstinspires.ftc.teamcode.versionTesting.trackingSS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSS;

@TeleOp(group = "Version Testing")
@Config
public class TrackingTestV1 extends LinearOpMode {
    public static Pose startPose = new Pose(0, 0, 0);
    public static boolean red = true;
    public static boolean trackingEnabled = false;
    public static boolean interpolEnabled = false;
    public static boolean shooterEnabled = false;
    boolean lastLB = false;
    boolean lastStart = false;
    boolean lastBack = false;
    boolean lastRB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        Servo turret1 = hardwareMap.get(Servo.class, "turret1");
        Servo turret2 = hardwareMap.get(Servo.class, "turret2");
        Servo hood1 = hardwareMap.get(Servo.class, "hood1");
        Servo hood2 = hardwareMap.get(Servo.class, "hood2");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocity(Globals.shooterInit);
        shooter2.setVelocity(Globals.shooterInit);
        turret1.setPosition(Globals.turretInit);
        turret2.setPosition(Globals.turretInit);
        hood1.setPosition(Globals.hoodInit);
        hood2.setPosition(Globals.hoodInit);
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get());
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get());
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get());
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get());
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()) {
            if (gamepad1.left_bumper && !lastLB) red = !red;
            lastLB = gamepad1.left_bumper;
            telemetry.addData("Alliance", red ? "RED" : "BLUE");
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;

        try {
            while (opModeIsActive()) {
                follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                follower.update();
                if (gamepad1.start && !lastStart) trackingEnabled = !trackingEnabled;
                if (gamepad1.back && !lastBack) interpolEnabled = !interpolEnabled;
                if (gamepad1.right_bumper && !lastRB) shooterEnabled = !shooterEnabled;
                lastStart = gamepad1.start;
                lastBack = gamepad1.back;
                lastRB = gamepad1.right_bumper;

                new TrackingSS(follower, red);
                TrackingSS.currentPollen = turret1.getPosition();
                TrackingSS.currentNectar = turret2.getPosition();
                if (trackingEnabled) {
                    turret1.setPosition(TrackingSS.getPollenTurretPos());
                    turret2.setPosition(TrackingSS.getNectarTurretPos());
                }
                TrackingSS.interpolate();
                hood1.setPosition(interpolEnabled ? TrackingSS.getTargetHoodPollen() : Globals.hoodShootPosPollen);
                hood2.setPosition(interpolEnabled ? TrackingSS.getTargetHoodNectar() : Globals.hoodShootPosNectar);
                shooter1.setVelocityPIDFCoefficients(Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
                shooter2.setVelocityPIDFCoefficients(Globals.kP_Nectar, Globals.kI_Nectar, Globals.kD_Nectar, Globals.kF_Nectar);
                double pollenTarget = shooterEnabled ? (interpolEnabled ? TrackingSS.getTargetRpmPollen() : Globals.shooterOnPollen) : Globals.shooterInit;
                double nectarTarget = shooterEnabled ? (interpolEnabled ? TrackingSS.getTargetRpmNectar() : Globals.shooterOnNectar) : Globals.shooterInit;
                shooter1.setVelocity(pollenTarget);
                shooter2.setVelocity(nectarTarget);

                telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
                telemetry.addData("Drive power FL/FR/BL/BR", frontLeft.getPower() + " / " + frontRight.getPower() + " / " + backLeft.getPower() + " / " + backRight.getPower());
                telemetry.addData("Alliance", red ? "RED" : "BLUE");
                telemetry.addData("Turret tracking", trackingEnabled);
                telemetry.addData("Interpolation", interpolEnabled);
                telemetry.addData("Shooter enabled", shooterEnabled);
                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading (degrees)", Math.toDegrees(follower.pose().heading()));
                telemetry.addData("Pollen turret position", turret1.getPosition());
                telemetry.addData("Nectar turret position", turret2.getPosition());
                telemetry.addData("Pollen turret angle (degrees)", TrackingSS.getPollenTurret());
                telemetry.addData("Nectar turret angle (degrees)", TrackingSS.getNectarTurret());
                telemetry.addData("Target dx", TrackingSS.dx);
                telemetry.addData("Target dy", TrackingSS.dy);
                telemetry.addData("Pollen target (ticks/s)", pollenTarget);
                telemetry.addData("Nectar target (ticks/s)", nectarTarget);
                telemetry.addData("Pollen velocity (ticks/s)", shooter1.getVelocity());
                telemetry.addData("Nectar velocity (ticks/s)", shooter2.getVelocity());
                telemetry.addData("Pollen current (A)", shooter1.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Nectar current (A)", shooter2.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Pollen hood", hood1.getPosition());
                telemetry.addData("Nectar hood", hood2.getPosition());
                telemetry.addData("Distance", TrackingSS.rangeVal);
                telemetry.addData("Reference distance", TrackingSS.referenceDist);
                telemetry.addData("Distance error", TrackingSS.rangeVal - TrackingSS.referenceDist);
                telemetry.addData("RPM per inch pollen", TrackingSS.rpmPerInchPollen);
                telemetry.addData("RPM per inch nectar", TrackingSS.rpmPerInchNectar);
                telemetry.addData("Hood per inch pollen", TrackingSS.hoodPerInchPollen);
                telemetry.addData("Hood per inch nectar", TrackingSS.hoodPerInchNectar);
                telemetry.update();
            }
        } finally {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
            follower.stop();
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
}
