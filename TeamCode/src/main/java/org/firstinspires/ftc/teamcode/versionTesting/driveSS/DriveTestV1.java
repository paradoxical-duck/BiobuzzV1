package org.firstinspires.ftc.teamcode.versionTesting.driveSS;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "Version Testing")
@Configurable
public class DriveTestV1 extends LinearOpMode {
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    public static double leftFrontPower = 0;
    public static double rightFrontPower = 0;
    public static double leftBackPower = 0;
    public static double rightBackPower = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        telemetry = new JoinedTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

            telemetry.addData("LF current", leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RF current", rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LB current", leftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RB current", rightBack.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
