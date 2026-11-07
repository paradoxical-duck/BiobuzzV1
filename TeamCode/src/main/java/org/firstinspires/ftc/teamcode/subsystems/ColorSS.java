
package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ColorSS {
    RobotHardware robot;
    private RevColorSensorV3 colorSensor;
    RobotHardware.Alliance alliance;

    public enum GameElements {POLLEN, NECTAR, NULL}

    public GameElements ball1 = GameElements.NULL;
    public GameElements ball2 = GameElements.NULL;
    public GameElements ball3 = GameElements.NULL;
    public GameElements ball4 = GameElements.NULL;

    public double hue = 0;
    public double saturation = 0;
    public double value = 0;
    public double distanceCm = 0;
    public double getHue() {
        return hue;
    }
    public double getSaturation() {
        return saturation;
    }
    public double getValue() {
        return value;
    }
    public double getDistance() {
        return distanceCm;
    }

    public ColorSS(RobotHardware robot, RobotHardware.Alliance alliance) {
        this.robot = robot;
        this.colorSensor = robot.colorSensor;
        this.alliance = alliance;
    }

    public ColorSS(RevColorSensorV3 colorSensor, RobotHardware.Alliance alliance) {
        this.colorSensor = colorSensor;
        this.alliance = alliance;
    }

    public void update(int ballCount) {
        distanceCm = colorSensor.getDistance(DistanceUnit.CM);

        if (ballCount > 0 && ballCount <= 4 && !robot.frontBeam.getState() && distanceCm <= Globals.colorMaxDistanceCm) {
            GameElements element = detectElement();

            if (element != GameElements.NULL) {
                switch (ballCount) {
                    case 1:
                        if (ball1 == GameElements.NULL) {
                            ball1 = element;
                        }
                        break;
                    case 2:
                        if (ball2 == GameElements.NULL) {
                            ball2 = element;
                        }
                        break;
                    case 3:
                        if (ball3 == GameElements.NULL) {
                            ball3 = element;
                        }
                        break;
                    case 4:
                        if (ball4 == GameElements.NULL) {
                            ball4 = element;
                        }
                        break;
                }
            }
        }
    }

    public GameElements detectElement() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        int max = Math.max(r, Math.max(g, b));

        if (max == 0) {
            return GameElements.NULL;
        }

        float[] hsv = new float[3];

        Color.RGBToHSV(
                Math.round(r * 255f / max),
                Math.round(g * 255f / max),
                Math.round(b * 255f / max),
                hsv
        );

        hue = hsv[0];
        saturation = hsv[1];
        value = hsv[2];

        if (saturation < Globals.colorMinSaturation) {
            return GameElements.NULL;
        }

        double nectarHue;
        double nectarTolerance;

        if (alliance == RobotHardware.Alliance.RED) {
            nectarHue = Globals.redNectarHue;
            nectarTolerance = Globals.redNectarHueTolerance;
        }
        else {
            nectarHue = Globals.blueNectarHue;
            nectarTolerance = Globals.blueNectarHueTolerance;
        }

        double pollenDifference = Math.abs(hue - Globals.pollenHue);
        pollenDifference = Math.min(pollenDifference, 360 - pollenDifference);

        double nectarDifference = Math.abs(hue - nectarHue);
        nectarDifference = Math.min(nectarDifference, 360 - nectarDifference);

        if (pollenDifference <= Globals.pollenHueTolerance && pollenDifference < nectarDifference) {
            return GameElements.POLLEN;
        }
        else if (nectarDifference <= nectarTolerance) {
            return GameElements.NECTAR;
        }

        return GameElements.NULL;
    }

    public boolean allPollen() {
        return ball1 == GameElements.POLLEN &&
                ball2 == GameElements.POLLEN &&
                ball3 == GameElements.POLLEN &&
                ball4 == GameElements.POLLEN;
    }

    public boolean allNectar() {
        return ball1 == GameElements.NECTAR &&
                ball2 == GameElements.NECTAR &&
                ball3 == GameElements.NECTAR &&
                ball4 == GameElements.NECTAR;
    }

    public void reset() {
        ball1 = GameElements.NULL;
        ball2 = GameElements.NULL;
        ball3 = GameElements.NULL;
        ball4 = GameElements.NULL;
    }
    public void logTelemetry(Telemetry telemetry) {
        telemetry.addLine("***Color SS***");
        telemetry.addData("Ball 1", ball1);
        telemetry.addData("Ball 2", ball2);
        telemetry.addData("Ball 3", ball3);
        telemetry.addData("Ball 4", ball4);
        telemetry.addData("Hue", hue);
        telemetry.addData("Hue", getHue());
        telemetry.addData("Saturation", getSaturation());
        telemetry.addData("Value", getValue());
        telemetry.addData("Distance (cm)", getDistance());
    }
    public void setDistance(double distanceCm) {
        this.distanceCm = distanceCm;
    }
}
