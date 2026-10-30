package org.firstinspires.ftc.teamcode.VersionTesting;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Configurable
@TeleOp
public class ShooterTestV1 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    public static double shooter1Vel = 0;
    public static double shooter2Vel = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.shooter1.setVelocityPIDFCoefficients(Globals.P, Globals.I, Globals.D, Globals.F);
            robot.shooter2.setVelocityPIDFCoefficients(Globals.P, Globals.I, Globals.D, Globals.F);

            robot.shooter1.setVelocity(shooter1Vel);
            robot.shooter2.setVelocity(shooter2Vel);

            telemetry.addData("shooter1 vel", shooter1Vel);
            telemetry.addData("shooter2 vel", shooter2Vel);
        }

    }
}
