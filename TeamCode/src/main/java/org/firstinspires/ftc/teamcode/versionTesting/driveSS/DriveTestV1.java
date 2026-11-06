package org.firstinspires.ftc.teamcode.versionTesting.driveSS;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(group = "Version Testing")
public class DriveTestV1 extends LinearOpMode {
    Follower follower;
    DcMotorEx lf, rf, lb, rb;
    double scale = 0.3;
    boolean lastA, lastB, lastX;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        lf = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get());
        rf = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get());
        lb = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get());
        rb = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get());
        telemetry.addLine("Left stick: translation | Right stick X: turn | A: 30% | B: 60% | X: 100%");
        telemetry.addLine("Hold RIGHT BUMPER to enable drive (dead-man control)");
        telemetry.update();
        waitForStart();
        try {
            while (opModeIsActive()) {
                if (gamepad1.a && !lastA) scale = 0.3;
                if (gamepad1.b && !lastB) scale = 0.6;
                if (gamepad1.x && !lastX) scale = 1;
                lastA = gamepad1.a; lastB = gamepad1.b; lastX = gamepad1.x;
                double power = gamepad1.right_bumper ? scale : 0;
                follower.manual(-gamepad1.left_stick_y * power, gamepad1.left_stick_x * power, gamepad1.right_stick_x * power);
                follower.update();
                telemetry.addLine("LEFT STICK: drive/strafe | RIGHT X: rotate | HOLD RB: ENABLE | A:30% B:60% X:100%");
                telemetry.addData("Drive enabled", gamepad1.right_bumper);
                telemetry.addData("Power scale", scale);
                telemetry.addData("LF ticks/s", lf.getVelocity()); telemetry.addData("LF A", lf.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("RF ticks/s", rf.getVelocity()); telemetry.addData("RF A", rf.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("LB ticks/s", lb.getVelocity()); telemetry.addData("LB A", lb.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("RB ticks/s", rb.getVelocity()); telemetry.addData("RB A", rb.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
                idle();
            }
        } finally {
            lf.setPower(0); rf.setPower(0); lb.setPower(0); rb.setPower(0);
        }
    }
}
