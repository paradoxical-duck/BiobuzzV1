
package org.firstinspires.ftc.teamcode.versionTesting.trackingSS;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSS;

@TeleOp(group = "Version Testing")
//todo tune TrackingSS values while running this
public class TrackingTestV1 extends LinearOpMode {
    TrackingSS trackingSS;
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    Servo turret1;
    Servo turret2;
    Servo hood1;
    Servo hood2;
    Follower follower;
    String alliance = "RED";
    ShooterSS.TurretState turretState = ShooterSS.TurretState.TRACKING_DISABLED;
    ShooterSS.ShooterState shooterState = ShooterSS.ShooterState.INTERPOL_DISABLED;
    ShooterSS.HoodState hoodState = ShooterSS.HoodState.INTERPOL_DISABLED;
    boolean lastLB = false;
    boolean red = true;
    boolean lastStart = false;
    boolean lastBack = false;
    boolean lastRB = false;
    public static Pose startPose = new Pose(0, 0, 0);
    public static boolean trackingEnabled = false, interpolEnabled = false, shooterEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        hood1 = hardwareMap.get(Servo.class, "hood1");
        hood2 = hardwareMap.get(Servo.class, "hood2");

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setVelocityPIDFCoefficients(Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
        shooter2.setVelocityPIDFCoefficients(Globals.kP_Nectar, Globals.kI_Nectar, Globals.kD_Nectar, Globals.kF_Nectar);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        while (opModeInInit()) {
            if (gamepad1.left_bumper && !lastLB) {
                red = !red;
            }
            lastLB = gamepad1.left_bumper;

            if (red) {
                alliance = "RED";
            }
            else {
                alliance = "BLUE";
            }

            telemetry.addLine("Initialized");
            telemetry.addLine(alliance + " alliance active");
            telemetry.update();
        }

        trackingSS = new TrackingSS(follower, red);

        waitForStart();

        while (opModeIsActive()) {
            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            follower.update();

            if (gamepad1.start && !lastStart) {
                trackingEnabled = !trackingEnabled;

                if (trackingEnabled) {
                    turret1.setPosition(TrackingSS.getTurretPos());
                    turret2.setPosition(turret1.getPosition());
                    turretState = ShooterSS.TurretState.TRACKING_ENABLED;
                }
                else {
                    turretState = ShooterSS.TurretState.TRACKING_DISABLED;
                }
            }
            lastStart = gamepad1.start;

            if (gamepad1.back && !lastBack) {
                interpolEnabled = !interpolEnabled;

                if (interpolEnabled) {
                    TrackingSS.interpolate();
                    hood1.setPosition(TrackingSS.getTargetHood());
                    hood2.setPosition(TrackingSS.getTargetHood());
                    if (shooterState == ShooterSS.ShooterState.INTERPOLATED) {
                        shooter1.setVelocity(TrackingSS.getTargetRpm());
                        shooter2.setVelocity(TrackingSS.getTargetRpm());
                    }
                    hoodState = ShooterSS.HoodState.INTERPOLATED;

                    if (shooterEnabled) {
                        shooter1.setVelocity(TrackingSS.getTargetRpm());
                        shooter2.setVelocity(TrackingSS.getTargetRpm());
                        shooterState = ShooterSS.ShooterState.INTERPOLATED;
                    }
                }
                else {
                    hood1.setPosition(Globals.hoodShootPosPollen);
                    hood2.setPosition(Globals.hoodShootPosNectar);
                    hoodState = ShooterSS.HoodState.SHOOT;

                    if (shooterEnabled) {
                        shooter1.setVelocity(Globals.shooterOnPollen);
                        shooter2.setVelocity(Globals.shooterOnNectar);
                        shooterState = ShooterSS.ShooterState.INTERPOL_DISABLED;
                    }
                }
            }
            lastBack = gamepad1.back;

            if (gamepad1.right_bumper && !lastRB) {
                shooterEnabled = !shooterEnabled;

                if (!shooterEnabled) {
                    shooter1.setVelocity(Globals.shooterInit);
                    shooter2.setVelocity(Globals.shooterInit);
                    shooterState = ShooterSS.ShooterState.INIT;
                }
                else if (interpolEnabled) {
                    shooter1.setVelocity(TrackingSS.getTargetRpm());
                    shooter2.setVelocity(TrackingSS.getTargetRpm());
                    shooterState = ShooterSS.ShooterState.INTERPOLATED;
                }
                else {
                    shooter1.setVelocity(Globals.shooterOnPollen);
                    shooter2.setVelocity(Globals.shooterOnNectar);
                    shooterState = ShooterSS.ShooterState.INTERPOL_DISABLED;
                }
            }
            lastRB = gamepad1.right_bumper;

            if (trackingEnabled) {
                turret1.setPosition(TrackingSS.getTurretPos());
                turret2.setPosition(turret1.getPosition());
            }

            if (interpolEnabled) {
                TrackingSS.interpolate();
                hood1.setPosition(TrackingSS.getTargetHood());
                hood2.setPosition(TrackingSS.getTargetHood());

                if (shooterEnabled) {
                    if (shooterState == ShooterSS.ShooterState.INTERPOLATED) {
                        shooter1.setVelocity(TrackingSS.getTargetRpm());
                        shooter2.setVelocity(TrackingSS.getTargetRpm());
                    }
                }
            }

            //*************************** telemetry ********************************
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Turret tracking", trackingEnabled);
            telemetry.addData("Interpolation", interpolEnabled);
            telemetry.addData("Shooter enabled", shooterEnabled);

            telemetry.addLine("");
            telemetry.addLine("***Robot pose***");
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", Math.toDegrees(follower.pose().heading()));

            telemetry.addLine("");
            telemetry.addLine("***Turret***");
            telemetry.addData("Turret state", turretState + " / " + turretState);
            telemetry.addData("Turret position", turret1.getPosition() + " / " + turret2.getPosition());
            telemetry.addData("Turret angle", TrackingSS.getTurret());
            telemetry.addData("Target dx", TrackingSS.dx);
            telemetry.addData("Target dy", TrackingSS.dy);

            telemetry.addLine("");
            telemetry.addLine("***Shooter***");
            telemetry.addData("Shooter state", shooterState + " / " + shooterState);
            telemetry.addData("Target velocity", TrackingSS.getTargetRpm());
            telemetry.addData("Shooter 1 velocity", shooter1.getVelocity());
            telemetry.addData("Shooter 2 velocity", shooter2.getVelocity());

            telemetry.addLine("");
            telemetry.addLine("***Hood***");
            telemetry.addData("Hood state", hoodState + " / " + hoodState);
            telemetry.addData("Target hood", TrackingSS.getTargetHood());
            telemetry.addData("Hood position", hood1.getPosition() + " / " + hood2.getPosition());

            telemetry.addLine("");
            telemetry.addLine("***Interpolation***");
            telemetry.addData("Distance", TrackingSS.rangeVal);
            telemetry.addData("Reference distance", TrackingSS.referenceDist);
            telemetry.addData("Distance error", TrackingSS.rangeVal - TrackingSS.referenceDist);
            telemetry.addData("RPM per inch", TrackingSS.rpmPerInchPollen);
            telemetry.addData("Hood per inch", TrackingSS.hoodPerInchPollen);

            telemetry.update();
        }
    }
}
