package org.firstinspires.ftc.teamcode.VersionTesting;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(group = "Version Testing")
@Configurable
public class IntakeTestV1 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    public static double intake1Pwr = 0;
    public static double intake2Pwr = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.intake1.setPower(intake1Pwr);
            robot.intake2.setPower(intake2Pwr);

            telemetry.addData("intake1 pwr", intake1Pwr);
            telemetry.addData("intake2 pwr", intake2Pwr);

            telemetry.addData("intake1 current",robot.intake1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("intake2 current",robot.intake2.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
