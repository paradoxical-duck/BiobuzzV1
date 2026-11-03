package org.firstinspires.ftc.teamcode.versionTesting.colorSS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ColorSS;

@TeleOp(group = "Version Testing")
public class ColorTestV1 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    ColorSS colorSS;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        colorSS = new ColorSS(robot, RobotHardware.alliance);

        waitForStart();

        while (opModeIsActive()) {
            colorSS.distanceCm = robot.colorSensor.getDistance(
                    org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM
            );
            colorSS.detectElement();

            telemetry.addData("Hue", colorSS.hue);
            telemetry.addData("Saturation", colorSS.saturation);
            telemetry.addData("Value", colorSS.value);
            telemetry.addData("Distance (cm)", colorSS.distanceCm);
            telemetry.addData("Detected element", colorSS.detectElement());
            telemetry.update();
        }
    }
}
