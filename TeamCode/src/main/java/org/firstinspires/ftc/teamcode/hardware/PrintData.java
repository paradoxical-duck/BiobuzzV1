package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ColorSS;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSS;

public class PrintData {
    private static final String[] sections = {
            "Critical Info",
            "Follower",
            "Intake",
            "Shooter",
            "Turret",
            "Hood",
            "Color",
            "Beambreakers"
    };

    private static final boolean[] enabled = new boolean[sections.length];
    private static int cursor = 0;

    public static void menu(Gamepad gamepad, Gamepad previousGamepad, Telemetry telemetry) {
        if (gamepad.dpad_down && !previousGamepad.dpad_down) {
            cursor = (cursor + 1) % sections.length;
        }

        if (gamepad.dpad_up && !previousGamepad.dpad_up) {
            cursor = (cursor - 1 + sections.length) % sections.length;
        }

        if (gamepad.dpad_right && !previousGamepad.dpad_right) {
            enabled[cursor] = !enabled[cursor];
        }

        telemetry.addLine("Dpad Up/Down: Select | Right: Toggle");

        for (int i = 0; i < sections.length; i++) {
            telemetry.addLine((i == cursor ? "> " : "  ") + (enabled[i] ? "[ON] " : "[OFF] ") + sections[i]);
        }
    }

    public static void print(RobotHardware robot, Follower follower, IntakeSS intakeSS, ShooterSS shooterSS, ColorSS colorSS, boolean trackingEnabled, boolean shooterEnabled, int ballCount, Telemetry telemetry) {
        if (enabled[0]) {
            telemetry.addLine("");
            telemetry.addLine("***Critical Info***");
            telemetry.addData("Alliance", RobotHardware.getAlliance());
            telemetry.addData("Tracking", trackingEnabled);
            telemetry.addData("Shooter", shooterEnabled);
            telemetry.addData("Intake", intakeSS.getIntakeState());
            telemetry.addData("Ball count", ballCount);
            telemetry.addData("Battery (V)", robot.voltage);
            telemetry.addData("Loop (ms)", robot.loopTimeMs);
        }

        if (enabled[1]) {
            telemetry.addLine("");
            telemetry.addLine("***Robot Pose***");
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", Math.toDegrees(follower.pose().heading()));
        }

        if (enabled[2]) {
            telemetry.addLine("");
            telemetry.addLine("***Intake***");
            telemetry.addData("Intake State", intakeSS.getIntakeState());
            telemetry.addData("Intake1 Current", robot.amps(robot.intake1));
            telemetry.addData("Intake2 Current", robot.amps(robot.intake2));
        }

        if (enabled[3]) {
            telemetry.addLine("");
            telemetry.addLine("***Shooter***");
            telemetry.addData("Shooter state (P/N)", shooterSS.getPollenShooterState() + " / " + shooterSS.getNectarShooterState());
            telemetry.addData("Target velocity pollen", TrackingSS.getTargetRpmPollen());
            telemetry.addData("Target velocity nectar", TrackingSS.getTargetRpmNectar());
            telemetry.addData("Shooter1 velocity", robot.shooter1.getVelocity());
            telemetry.addData("Shooter1 current", robot.amps(robot.shooter1));
            telemetry.addData("Shooter2 velocity", robot.shooter2.getVelocity());
            telemetry.addData("Shooter2 current", robot.amps(robot.shooter2));
        }

        if (enabled[4]) {
            telemetry.addLine("");
            telemetry.addLine("***Turret***");
            telemetry.addData("Turret state (P/N)", shooterSS.getPollenTurretState() + " / " + shooterSS.getNectarTurretState());
            telemetry.addData("Turret position (P/N)", robot.turret1.getPosition() + " / " + robot.turret2.getPosition());
            telemetry.addData("Turret angle pollen", TrackingSS.getPollenTurret());
            telemetry.addData("Turret angle nectar", TrackingSS.getNectarTurret());
        }

        if (enabled[5]) {
            telemetry.addLine("");
            telemetry.addLine("***Hood***");
            telemetry.addData("Hood state (P/N)", shooterSS.getPollenHoodState() + " / " + shooterSS.getNectarHoodState());
            telemetry.addData("Target hood pollen", TrackingSS.getTargetHoodPollen());
            telemetry.addData("Target hood nectar", TrackingSS.getTargetHoodNectar());
            telemetry.addData("Hood position (P/N)", robot.hood1.getPosition() + " / " + robot.hood2.getPosition());
        }

        if (enabled[6]) {
            telemetry.addLine("");
            telemetry.addLine("***Color SS***");
            telemetry.addData("Ball 1", colorSS.ball1);
            telemetry.addData("Ball 2", colorSS.ball2);
            telemetry.addData("Ball 3", colorSS.ball3);
            telemetry.addData("Ball 4", colorSS.ball4);
            telemetry.addData("Hue", colorSS.getHue());
            telemetry.addData("Saturation", colorSS.getSaturation());
            telemetry.addData("Value", colorSS.getValue());
            telemetry.addData("Distance (cm)", colorSS.getDistance());
        }

        if (enabled[7]) {
            telemetry.addLine("");
            telemetry.addLine("***Beambreakers***");
            telemetry.addData("Front", robot.frontBeam.getState());
            telemetry.addData("Mid", robot.midBeam.getState());
            telemetry.addData("Pollen", robot.pollenBeam.getState());
            telemetry.addData("Nectar", robot.nectarBeam.getState());
            telemetry.addData("Ball count", ballCount);
        }
    }
}