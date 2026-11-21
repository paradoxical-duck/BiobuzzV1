package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@Configurable
@TeleOp( group = "Version Testing")
public class ShooterTestV2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

            while (opModeIsActive()) {
                shooter.setPower(1);

                telemetry.addData("current", shooter.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
            }
    }
}