package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Configurable
@TeleOp(group = "Version Testing")
public class LockTestV1 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    public static double lock1Pos = 0;
    public static double lock2Pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.lock1.setPosition(lock1Pos);
            robot.lock2.setPosition(lock2Pos);
            telemetry.addData("lock1 pos", lock1Pos);
            telemetry.addData("lock2 pos", lock2Pos);
            telemetry.update();
        }
    }
}
