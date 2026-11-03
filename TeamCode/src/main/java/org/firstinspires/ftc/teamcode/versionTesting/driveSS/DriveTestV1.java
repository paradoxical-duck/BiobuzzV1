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
    DcMotorEx m1;
    DcMotorEx m2;
    DcMotorEx m3;
    DcMotorEx m4;
    public static double m1Power = 0;
    public static double m2Power = 0;
    public static double m3Power = 0;
    public static double m4Power = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m3 = hardwareMap.get(DcMotorEx.class, "m2");
        m2 = hardwareMap.get(DcMotorEx.class, "m3");
        m4 = hardwareMap.get(DcMotorEx.class, "m4");

        telemetry = new JoinedTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            m1.setPower(m1Power);
            m3.setPower(m2Power);
            m2.setPower(m3Power);
            m4.setPower(m4Power);

            telemetry.addData("m1 current", m1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("m2 current", m3.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("m3 current", m2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("m4 current", m4.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
