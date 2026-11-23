package org.firstinspires.ftc.teamcode.versionTesting.followerTesting;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Follower Test", group = "Version Testing")
public class FollowerTest extends LinearOpMode {
    Follower follower;
    boolean fieldCentric;
    boolean lastA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);

        while (opModeInInit()) {
            if (gamepad1.a && !lastA) {
                fieldCentric = !fieldCentric;
            }
            lastA = gamepad1.a;

            telemetry.addLine("A: toggle field centric");
            telemetry.addData("Field centric", fieldCentric);
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            if (!fieldCentric) {
                follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            else {
                DrivePowers powers = ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        follower.pose().heading()
                );
                follower.manual(powers);
            }
            follower.update();

            telemetry.addData("Field centric", fieldCentric);
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }
    }
}
