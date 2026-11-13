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

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        colorSS = new ColorSS(colorSensor, RobotHardware.getAlliance());
        waitForStart();
        while (opModeIsActive()) {
            colorSS.distanceCm = colorSensor.getDistance(DistanceUnit.CM);
            String element = String.valueOf(colorSS.detectElement());

            telemetry.addLine("TUNING INSTRUCTIONS");
            telemetry.addLine("* Take multiple readings of each ball");
            telemetry.addLine("* Use median of readings for hue");
            telemetry.addLine("* Use range of readings for tolerance");
            telemetry.addLine("* Keep ball at extreme position and get max distance");
            telemetry.addLine("* Take lowest saturation and reduce it even further to get min saturation");

            colorSS.logTelemetry(telemetry);
            telemetry.addData("Detected element", element);

            telemetry.update();
            idle();
        }
    }
}
