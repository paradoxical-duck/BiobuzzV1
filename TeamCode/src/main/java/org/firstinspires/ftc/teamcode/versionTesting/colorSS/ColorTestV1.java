package org.firstinspires.ftc.teamcode.versionTesting.colorSS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.subsystems.ColorSS;

@TeleOp(group = "Version Testing")
@Config
public class ColorTestV1 extends LinearOpMode {
    public static boolean red = true;

    @Override
    public void runOpMode() {
        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            ColorSS colorSS = new ColorSS(colorSensor, red);
            colorSS.distanceCm = colorSensor.getDistance(DistanceUnit.CM);
            ColorSS.GameElements element = colorSS.detectElement();
            telemetry.addData("Alliance", red ? "RED" : "BLUE");
            telemetry.addData("Hue", colorSS.hue);
            telemetry.addData("Saturation", colorSS.saturation);
            telemetry.addData("Value", colorSS.value);
            telemetry.addData("Distance (cm)", colorSS.distanceCm);
            telemetry.addData("Detected element", colorSS.distanceCm <= Globals.colorMaxDistanceCm ? element : ColorSS.GameElements.NULL);
            telemetry.update();
        }
    }
}
