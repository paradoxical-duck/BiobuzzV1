package org.firstinspires.ftc.teamcode.versionTesting.trackingSS;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSS;

@TeleOp(group = "Version Testing")
public class TrackingTestV1 extends LinearOpMode {
    Follower follower;
    DcMotorEx shooter1, shooter2;
    Servo turret1, turret2, hood1, hood2;
    boolean red = true, trackingEnabled = false, interpolEnabled = false, shooterEnabled = false;
    boolean lastLB, lastRB, lastStart, lastBack, lastLeft, lastRight, lastUp, lastDown, lastA, lastB, lastX;
    boolean fine = false;
    int selected = 0;
    double driveScale = 0.3;
    public static Pose startPose = new Pose(0, 0, 0);
    String[] labels = {"Pollen base RPM", "Nectar base RPM", "Pollen base hood", "Nectar base hood", "Pollen RPM/in", "Nectar RPM/in", "Pollen hood/in", "Nectar hood/in", "Pollen turret offset deg", "Nectar turret offset deg", "Pollen turret kP", "Nectar turret kP", "Pollen turret kS", "Nectar turret kS"};

    double getValue(int index) {
        switch (index) {
            case 0: return Globals.shooterOnPollen;
            case 1: return Globals.shooterOnNectar;
            case 2: return Globals.hoodShootPosPollen;
            case 3: return Globals.hoodShootPosNectar;
            case 4: return TrackingSS.rpmPerInchPollen;
            case 5: return TrackingSS.rpmPerInchNectar;
            case 6: return TrackingSS.hoodPerInchPollen;
            case 7: return TrackingSS.hoodPerInchNectar;
            case 8: return Globals.turretOffsetPollen;
            case 9: return Globals.turretOffsetNectar;
            case 10: return TrackingSS.kP_Pollen;
            case 11: return TrackingSS.kP_Nectar;
            case 12: return TrackingSS.kS_Pollen;
            default: return TrackingSS.kS_Nectar;
        }
    }

    void setValue(int index, double value) {
        switch (index) {
            case 0: Globals.shooterOnPollen = Math.max(0, value); break;
            case 1: Globals.shooterOnNectar = Math.max(0, value); break;
            case 2: Globals.hoodShootPosPollen = Range.clip(value, 0, 1); break;
            case 3: Globals.hoodShootPosNectar = Range.clip(value, 0, 1); break;
            case 4: TrackingSS.rpmPerInchPollen = value; break;
            case 5: TrackingSS.rpmPerInchNectar = value; break;
            case 6: TrackingSS.hoodPerInchPollen = value; break;
            case 7: TrackingSS.hoodPerInchNectar = value; break;
            case 8: Globals.turretOffsetPollen = value; break;
            case 9: Globals.turretOffsetNectar = value; break;
            case 10: TrackingSS.kP_Pollen = Math.max(0, value); break;
            case 11: TrackingSS.kP_Nectar = Math.max(0, value); break;
            case 12: TrackingSS.kS_Pollen = Math.max(0, value); break;
            case 13: TrackingSS.kS_Nectar = Math.max(0, value); break;
        }
    }

    double getStep() {
        if (selected < 2) return fine ? 10 : 50;
        if (selected < 4) return fine ? 0.001 : 0.01;
        if (selected < 6) return fine ? 0.1 : 1;
        if (selected < 8) return fine ? 0.0001 : 0.001;
        if (selected < 10) return fine ? 0.1 : 1;
        return fine ? 0.0001 : 0.001;
    }

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
        TrackingSS.currentPollen = turret1.getPosition();
        TrackingSS.currentNectar = turret2.getPosition();
        while (opModeInInit()) {
            if (gamepad1.left_bumper && !lastLB) red = !red;
            lastLB = gamepad1.left_bumper;
            telemetry.addLine("INIT: LB toggles RED/BLUE | Drive requires RIGHT TRIGGER held when running");
            telemetry.addData("Alliance", red ? "RED" : "BLUE");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        new TrackingSS(follower, red);
        lastLB = gamepad1.left_bumper;
        try {
            while (opModeIsActive()) {
                double power = gamepad1.right_trigger > 0.5 ? driveScale : 0;
                follower.manual(-gamepad1.left_stick_y * power, gamepad1.left_stick_x * power, gamepad1.right_stick_x * power);
                follower.update();
                if (gamepad1.start && !lastStart) trackingEnabled = !trackingEnabled;
                if (gamepad1.back && !lastBack) interpolEnabled = !interpolEnabled;
                if (gamepad1.right_bumper && !lastRB) shooterEnabled = !shooterEnabled;
                if (gamepad1.dpad_left && !lastLeft) selected = (selected + labels.length - 1) % labels.length;
                if (gamepad1.dpad_right && !lastRight) selected = (selected + 1) % labels.length;
                if (gamepad1.a && !lastA) fine = false;
                if (gamepad1.b && !lastB) fine = true;
                if (gamepad1.dpad_up && !lastUp) setValue(selected, getValue(selected) + getStep());
                if (gamepad1.dpad_down && !lastDown) setValue(selected, getValue(selected) - getStep());
                if (gamepad1.x && !lastX) shooterEnabled = false;
                lastStart = gamepad1.start; lastBack = gamepad1.back; lastRB = gamepad1.right_bumper;
                lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
                lastA = gamepad1.a; lastB = gamepad1.b;
                lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
                lastX = gamepad1.x;
                if (trackingEnabled) {
                    TrackingSS.currentPollen = turret1.getPosition();
                    TrackingSS.currentNectar = turret2.getPosition();
                    turret1.setPosition(TrackingSS.getPollenTurretPos());
                    turret2.setPosition(TrackingSS.getNectarTurretPos());
                }
                TrackingSS.interpolate();
                hood1.setPosition(interpolEnabled ? TrackingSS.getTargetHoodPollen() : Globals.hoodShootPosPollen);
                hood2.setPosition(interpolEnabled ? TrackingSS.getTargetHoodNectar() : Globals.hoodShootPosNectar);
                shooter1.setVelocity(shooterEnabled ? (interpolEnabled ? TrackingSS.getTargetRpmPollen() : Globals.shooterOnPollen) : 0);
                shooter2.setVelocity(shooterEnabled ? (interpolEnabled ? TrackingSS.getTargetRpmNectar() : Globals.shooterOnNectar) : 0);
                telemetry.addLine("INIT LB: alliance | START: turret tracking | BACK: interpolation | RB: shooters toggle | X: SHOOTERS OFF");
                telemetry.addLine("LEFT/RIGHT: parameter | UP/DOWN: +/- | A: coarse | B: fine | Hold RT: drive (30%)");
                telemetry.addData("Alliance", red ? "RED" : "BLUE");
                telemetry.addData("Turret tracking", trackingEnabled);
                telemetry.addData("Interpolation", interpolEnabled);
                telemetry.addData("Shooters enabled", shooterEnabled);
                telemetry.addData("Selected parameter", labels[selected]);
                telemetry.addData("Parameter value", getValue(selected));
                telemetry.addData("Step", getStep());
                telemetry.addData("Robot pose x/y/deg", "%.2f / %.2f / %.2f", follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
                telemetry.addData("Pollen turret/angle", "%.5f / %.2f", turret1.getPosition(), TrackingSS.getPollenTurret());
                telemetry.addData("Nectar turret/angle", "%.5f / %.2f", turret2.getPosition(), TrackingSS.getNectarTurret());
                telemetry.addData("Pollen target/actual ticks/s", "%.1f / %.1f", shooterEnabled ? (interpolEnabled ? TrackingSS.getTargetRpmPollen() : Globals.shooterOnPollen) : 0, shooter1.getVelocity());
                telemetry.addData("Nectar target/actual ticks/s", "%.1f / %.1f", shooterEnabled ? (interpolEnabled ? TrackingSS.getTargetRpmNectar() : Globals.shooterOnNectar) : 0, shooter2.getVelocity());
                telemetry.addData("Pollen hood", hood1.getPosition());
                telemetry.addData("Nectar hood", hood2.getPosition());
                telemetry.addData("Distance / reference", "%.2f / %.2f", TrackingSS.rangeVal, TrackingSS.referenceDist);
                telemetry.update();
                idle();
            }
        } finally {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }
    }
}
