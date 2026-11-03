package org.firstinspires.ftc.teamcode.versionTesting.colorSS;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ColorSS;

@TeleOp(group = "Version Testing")
@Config
public class ColorTestV1 extends LinearOpMode {
    ColorSS colorSS;

    RevColorSensorV3 colorSensor;
    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        colorSS = new ColorSS(colorSensor, true);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            colorSS.distanceCm = colorSensor.getDistance(
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
