package org.firstinspires.ftc.teamcode.versionTesting.intakeSS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "Version Testing")
public class IntakeTestV1 extends LinearOpMode {
    DcMotorEx intake1;
    DcMotorEx intake2;
    public static double intake1Pwr = 1;
    public static double intake2Pwr = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        waitForStart();
        while (opModeIsActive()) {
            intake1.setPower(intake1Pwr);
            intake2.setPower(intake2Pwr);

            telemetry.addData("intake1 pwr", intake1Pwr);
            telemetry.addData("intake2 pwr", intake2Pwr);

            telemetry.addData("intake1 current", intake1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("intake2 current", intake2.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
