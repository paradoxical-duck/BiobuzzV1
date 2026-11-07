package org.firstinspires.ftc.teamcode.versionTesting.driveSS;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Para Drive Test I 💛", group = "AAAAAAAAA")
public class DriveTestV2 extends LinearOpMode {
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
            follower.manual(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            follower.update();

            telemetry.addData("LF current", amps(lf));
            telemetry.addData("LF power", power(lf));

            telemetry.addData("RF current", amps(rf));
            telemetry.addData("RF power", power(rf));

            telemetry.addData("LB current", amps(lb));
            telemetry.addData("LB power", power(lb));

            telemetry.addData("RB current", amps(rb));
            telemetry.addData("RF power", power(rb));
            telemetry.update();
        }

    }
    public double amps(DcMotorEx motor) { return motor.getCurrent(CurrentUnit.AMPS); }
    public double power(DcMotorEx motor) { return motor.getPower(); }
}
