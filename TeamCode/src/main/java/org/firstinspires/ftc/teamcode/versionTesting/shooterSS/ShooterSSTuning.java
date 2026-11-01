package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(group = "Version Testing")
@Configurable
public class ShooterSSTuning extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    public static double shooterVel = 0;
    public static double turretPos = 0;
    public static double hoodPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            robot.shooter1.setVelocity(shooterVel);
            robot.shooter2.setVelocity(shooterVel);
            robot.turret.setPosition(turretPos);
            robot.hood.setPosition(hoodPos);

            telemetry.addData("shooter vel", shooterVel);
            telemetry.addData("turret pos", turretPos);
            telemetry.addData("hood pos", hoodPos);
            telemetry.update();
        }
    }
}
