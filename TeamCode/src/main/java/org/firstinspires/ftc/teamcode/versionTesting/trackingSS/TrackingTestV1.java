package org.firstinspires.ftc.teamcode.versionTesting.trackingSS;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSS;

@TeleOp(group = "Version Testing")
public class TrackingTestV1 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    IntakeSS intakeSS;
    Follower follower;
    DcMotorEx shooter1;
    Servo turret1, hood1;

    boolean red = true;
    boolean trackingEnabled = false;
    boolean interpolEnabled = false;
    boolean shooterEnabled = false;
    boolean intakeEnabled = false;

    boolean lastLB, lastRB, lastStart, lastBack, lastLeft, lastRight, lastUp, lastDown, lastA, lastX, lastY;

    int selected = 0;
    int hiveMode = 0;
    double step = 0.01;

    public static Pose startPose = new Pose(0, 0, 0);

    String[] labels = {
            "Pollen base power",
            "Pollen base hood",
            "Pollen power/in",
            "Pollen hood/in",
            "Pollen turret offset deg"
    };

    double getValue(int index) {
        switch (index) {
            case 0: return Globals.shooterOnPollen;
            case 1: return Globals.hoodShootPosPollen;
            case 2: return TrackingSS.rpmPerInchPollen;
            case 3: return TrackingSS.hoodPerInchPollen;
            default: return Globals.turretOffsetPollen;
        }
    }

    void setValue(int index, double value) {
        switch (index) {
            case 0: Globals.shooterOnPollen = Range.clip(value, 0, 1); break;
            case 1: Globals.hoodShootPosPollen = Range.clip(value, 0, 1); break;
            case 2: TrackingSS.rpmPerInchPollen = value; break;
            case 3: TrackingSS.hoodPerInchPollen = value; break;
            case 4: Globals.turretOffsetPollen = value; break;
        }
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        intakeSS = new IntakeSS(robot);

        shooter1 = robot.shooter1;
        turret1 = robot.turret1;
        hood1 = robot.hood1;

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeSS.updateState(IntakeSS.IntakeState.INIT);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        TrackingSS.currentPollen = turret1.getPosition();

        while (opModeInInit()) {
            if (gamepad1.left_bumper && !lastLB) red = !red;

            lastLB = gamepad1.left_bumper;

            telemetry.addLine("INIT: LEFT BUMPER toggles alliance");
            telemetry.addData("Alliance", red ? "RED" : "BLUE");
            telemetry.addData("Start X", startPose.x());
            telemetry.addData("Start Y", startPose.y());
            telemetry.addData("Start heading", Math.toDegrees(startPose.heading()));
            telemetry.update();
        }

        RobotHardware.setAlliance(red ? RobotHardware.Alliance.RED : RobotHardware.Alliance.BLUE);

        new TrackingSS(follower, red);

        TrackingSS.overrideHive = false;

        waitForStart();

        lastLB = gamepad1.left_bumper;

        while (opModeIsActive()) {
            robot.clearBulkCache();

            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            follower.update();

            if (gamepad1.start && !lastStart) trackingEnabled = !trackingEnabled;
            if (gamepad1.back && !lastBack) interpolEnabled = !interpolEnabled;
            if (gamepad1.a && !lastA) shooterEnabled = !shooterEnabled;

            if (gamepad1.left_bumper && !lastLB) selected = (selected + 1) % labels.length;

            if (gamepad1.right_bumper && !lastRB) {
                intakeEnabled = !intakeEnabled;
                intakeSS.updateState(intakeEnabled ? IntakeSS.IntakeState.ON : IntakeSS.IntakeState.INIT);
            }

            if (gamepad1.y && !lastY) {
                hiveMode = (hiveMode + 1) % 3;
            }

            TrackingSS.overrideHive = hiveMode != 0;
            TrackingSS.overrideUpperHive = hiveMode == 2;

            if (gamepad1.dpad_left && !lastLeft) step /= 10;
            if (gamepad1.dpad_right && !lastRight) step *= 10;

            if (gamepad1.dpad_up && !lastUp) setValue(selected, getValue(selected) + step);
            if (gamepad1.dpad_down && !lastDown) setValue(selected, getValue(selected) - step);

            if (gamepad1.x && !lastX) shooterEnabled = false;

            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;
            lastStart = gamepad1.start;
            lastBack = gamepad1.back;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastA = gamepad1.a;
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            if (trackingEnabled) {
                turret1.setPosition(TrackingSS.getPollenTurretPos());
            }
            else {
                TrackingSS.setDxAndDy();
                TrackingSS.currentPollen = turret1.getPosition();
            }

            TrackingSS.interpolate();

            double targetPower = shooterEnabled ? Range.clip(interpolEnabled ? TrackingSS.getTargetRpmPollen() : Globals.shooterOnPollen, 0, 1) : 0;
            double targetHood = Range.clip(interpolEnabled ? TrackingSS.getTargetHoodPollen() : Globals.hoodShootPosPollen, 0, 1);

            shooter1.setPower(targetPower);
            hood1.setPosition(targetHood);

            telemetry.addLine("******** CONTROLS ********");
            telemetry.addLine("START: Tracking | BACK: Interpolation");
            telemetry.addLine("Y: AUTO / DOWN / UP hive selection");
            telemetry.addLine("LB: Next parameter | RB: Intake");
            telemetry.addLine("DPAD LEFT/RIGHT: Step /10 or x10");
            telemetry.addLine("DPAD UP/DOWN: Change parameter");
            telemetry.addLine("A: Shooter toggle | X: Shooter off");

            telemetry.addLine("******** STATUS ********");
            telemetry.addData("Alliance", red ? "RED" : "BLUE");
            telemetry.addData("Tracking", trackingEnabled);
            telemetry.addData("Interpolation", interpolEnabled);
            telemetry.addData("Shooter", shooterEnabled);
            telemetry.addData("Intake", intakeSS.getIntakeState());
            telemetry.addData("Hive mode", hiveMode == 0 ? "AUTO" : hiveMode == 1 ? "DOWN" : "UP");
            telemetry.addData("Selected hive", TrackingSS.upperHive ? "UP" : "DOWN");

            telemetry.addLine("******** TRACKING DIAGNOSTICS ********");
            telemetry.addData("Robot X", follower.pose().x());
            telemetry.addData("Robot Y", follower.pose().y());
            telemetry.addData("Robot heading (deg)", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Hive X", TrackingSS.goalX);
            telemetry.addData("Hive Y", TrackingSS.goalY);
            telemetry.addData("Field target bearing (deg)", TrackingSS.targetHeading);
            telemetry.addData("Robot heading used (deg)", TrackingSS.robotHeading);
            telemetry.addData("Requested turret angle (deg)", TrackingSS.requestedAnglePollen);
            telemetry.addData("Reachable turret angle (deg)", TrackingSS.targetAnglePollen);
            telemetry.addData("Target reachable", TrackingSS.reachablePollen);
            telemetry.addData("Servo target", TrackingSS.targetServoPollen);
            telemetry.addData("Servo command", turret1.getPosition());
            telemetry.addData("Calculated turret angle (deg)", TrackingSS.getPollenTurret());
//            telemetry.addData("Command angular error (deg)", TrackingSS.errorAnglePollen);

            telemetry.addLine("******** TUNING ********");
            telemetry.addData("Selected", labels[selected]);
            telemetry.addData("Step", step);
            telemetry.addData("Selected value", getValue(selected));

            for (int i = 0; i < labels.length; i++) {
                telemetry.addData(labels[i], getValue(i));
            }

            telemetry.addLine("******** SHOOTER ********");
            telemetry.addData("Target power", targetPower);
            telemetry.addData("Current power", shooter1.getPower());
            telemetry.addData("Velocity (ticks/s)", shooter1.getVelocity());
            telemetry.addData("Current (A)", shooter1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Hood", hood1.getPosition());

            telemetry.addLine("******** DISTANCE ********");
            telemetry.addData("Distance to hive (in)", TrackingSS.rangeVal);
            telemetry.addData("Reference distance (in)", TrackingSS.referenceDist);
            telemetry.addData("Distance error (in)", TrackingSS.rangeVal - TrackingSS.referenceDist);

            telemetry.update();
        }

        shooter1.setPower(0);
        intakeSS.updateState(IntakeSS.IntakeState.INIT);
        TrackingSS.overrideHive = false;
    }
}