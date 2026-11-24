package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;

@TeleOp(group = "Version Testing")
public class ShooterSSTuning extends LinearOpMode
/**todo use this to tune shooter, hood, turret values at auto shoot pose before S3*/
{
    RobotHardware robot = RobotHardware.getInstance();
    IntakeSS intakeSS;
    DcMotorEx shooter;
    Servo turret, hood;

    double velocity = 0; // ticks/s
    double turretPos, hoodPos;
    int selected = 0;
    boolean intakeEnabled = false;
    boolean lastLB, lastRB, lastUp, lastDown, lastX;

    String[] names = {"Pollen Velocity", "Pollen Turret", "Pollen Hood"};

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        intakeSS = new IntakeSS(robot);

        shooter = robot.shooter1;
        turret = robot.turret1;
        hood = robot.hood1;

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretPos = turret.getPosition();
        hoodPos = hood.getPosition();

        intakeSS.updateState(IntakeSS.IntakeState.INIT);

        waitForStart();

        while (opModeIsActive()) {
            robot.clearBulkCache();

            if (gamepad1.left_bumper && !lastLB) selected = (selected + 1) % 3;

            if (gamepad1.right_bumper && !lastRB) {
                intakeEnabled = !intakeEnabled;
                intakeSS.updateState(intakeEnabled ? IntakeSS.IntakeState.ON : IntakeSS.IntakeState.INIT);
            }

            if (gamepad1.dpad_up && !lastUp) {
                if (selected == 0) velocity += 50;
                if (selected == 1) turretPos = Range.clip(turretPos + 0.01, 0, 1);
                if (selected == 2) hoodPos = Range.clip(hoodPos + 0.01, 0, 1);
            }

            if (gamepad1.dpad_down && !lastDown) {
                if (selected == 0) velocity = Math.max(0, velocity - 50);
                if (selected == 1) turretPos = Range.clip(turretPos - 0.01, 0, 1);
                if (selected == 2) hoodPos = Range.clip(hoodPos - 0.01, 0, 1);
            }

            if (gamepad1.x && !lastX) velocity = 0;

            shooter.setVelocity(velocity);
            turret.setPosition(turretPos);
            hood.setPosition(hoodPos);

            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastX = gamepad1.x;

            telemetry.addLine("LEFT BUMPER: Next parameter");
            telemetry.addLine("RIGHT BUMPER: Toggle intake");
            telemetry.addLine("DPAD UP/DOWN: Increase/decrease");
            telemetry.addLine("X: Stop shooter");

            telemetry.addData("Selected", names[selected]);
            telemetry.addData("Velocity step (ticks/s)", 50);
            telemetry.addData("Servo step", 0.01);
            telemetry.addData("Intake", intakeSS.getIntakeState());

            telemetry.addLine("******** POLLEN ********");
            telemetry.addData("Target velocity (ticks/s)", velocity);
            telemetry.addData("Shooter velocity (ticks/s)", shooter.getVelocity());
            telemetry.addData("Shooter current (A)", shooter.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Turret getPosition()", turret.getPosition());
            telemetry.addData("Hood getPosition()", hood.getPosition());

            telemetry.update();
            idle();
        }

        shooter.setVelocity(0);
        intakeSS.updateState(IntakeSS.IntakeState.INIT);
    }
}