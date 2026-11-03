package org.firstinspires.ftc.teamcode.versionTesting.intakeSS;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(group = "Version Testing")
@Configurable
public class IntakeDeployTestV1 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    public static double intakeDeployPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.intakeDeploy.setPosition(intakeDeployPos);

            telemetry.addData("target pos", intakeDeployPos);
            telemetry.addData("current pos", intakeDeployPos);
            telemetry.update();
        }
    }
}
