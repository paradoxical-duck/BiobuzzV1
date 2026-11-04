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
    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;
    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        lf = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get());
        rf = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get());
        lb = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get());
        rb = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get());
        waitForStart();
        while (opModeIsActive()) {
            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            follower.update();

            telemetry.addData("LF velocity", lf.getVelocity());
            telemetry.addData("LF current", lf.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("RF velocity", rf.getVelocity());
            telemetry.addData("RF current", rf.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("LB velocity", lb.getVelocity());
            telemetry.addData("LB current", lb.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("RB velocity", rb.getVelocity());
            telemetry.addData("RB current", rb.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
