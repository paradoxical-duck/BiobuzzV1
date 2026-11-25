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

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (Globals.kP_Pollen != 0 || Globals.kI_Pollen != 0 || Globals.kD_Pollen != 0 || Globals.kF_Pollen != 0) {
            shooter.setVelocityPIDFCoefficients(Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
        }

        waitForStart();

        while (opModeIsActive()) {
            shooter.setVelocity(Math.max(0, Globals.shooterOnPollen));

            telemetry.addData("Target velocity (ticks/s)", Globals.shooterOnPollen);
            telemetry.addData("Actual velocity (ticks/s)", shooter.getVelocity());

            telemetry.addData("current", shooter.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
        shooter.setVelocity(0);
    }
}