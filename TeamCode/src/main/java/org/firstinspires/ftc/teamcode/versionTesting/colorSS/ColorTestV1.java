package org.firstinspires.ftc.teamcode.versionTesting.colorSS;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ColorSS;

@TeleOp(group = "Version Testing")
public class ColorTestV1 extends LinearOpMode {
    ColorSS colorSS;
    RevColorSensorV3 colorSensor;
    boolean red = true, lastX;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        colorSS = new ColorSS(colorSensor, RobotHardware.getAlliance());
        telemetry.addLine("X: toggle RED/BLUE alliance | All sensor values update continuously");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x && !lastX) {
                red = !red;
                colorSS = new ColorSS(colorSensor, RobotHardware.getAlliance());
            }
            lastX = gamepad1.x;
            colorSS.distanceCm = colorSensor.getDistance(DistanceUnit.CM);
            String element = String.valueOf(colorSS.detectElement());
            telemetry.addLine("X: toggle alliance (RED/BLUE)");
            telemetry.addData("Alliance", red ? "RED" : "BLUE");
            telemetry.addData("Hue", colorSS.hue);
            telemetry.addData("Saturation", colorSS.saturation);
            telemetry.addData("Value", colorSS.value);
            telemetry.addData("Distance (cm)", colorSS.distanceCm);
            telemetry.addData("Detected element", element);
            telemetry.update();
            idle();
        }
    }
}
