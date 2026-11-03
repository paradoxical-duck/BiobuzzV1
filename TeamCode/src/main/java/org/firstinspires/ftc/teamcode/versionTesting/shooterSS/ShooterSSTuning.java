package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(group = "Version Testing")
@Configurable
public class ShooterSSTuning extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    public static double shooter1Vel = 0;
    public static double turret1Pos = 0;
    public static double hood1Pos = 0;

    public static double shooter2Vel = 0;
    public static double turret2Pos = 0;
    public static double hood2Pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            robot.shooter1.setVelocityPIDFCoefficients(Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
            robot.shooter2.setVelocityPIDFCoefficients(Globals.kP_Nectar, Globals.kI_Nectar, Globals.kD_Nectar, Globals.kF_Nectar);


            robot.shooter1.setVelocity(shooter1Vel);
            robot.turret1.setPosition(turret1Pos);
            robot.hood1.setPosition(hood1Pos);


            robot.shooter2.setVelocity(shooter2Vel);
            robot.turret2.setPosition(turret2Pos);
            robot.hood2.setPosition(hood2Pos);

            telemetry.addData("shooter1 vel", shooter1Vel);
            telemetry.addData("turret1 pos", turret1Pos);
            telemetry.addData("hood1 pos", hood1Pos);
            telemetry.addData("shooter2 vel", shooter2Vel);
            telemetry.addData("turret2 pos", turret2Pos);
            telemetry.addData("hood2 pos", hood2Pos);
            telemetry.update();
        }
    }
}
