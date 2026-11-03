package org.firstinspires.ftc.teamcode.hardware;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PrintData {
    public enum Section { LOOP, POSE, BEAMS, INTAKE, SHOOTER, TURRET_HOOD, COLOR }

    private static final boolean[] on = new boolean[Section.values().length]; // all off
    private static int cursor = 0;

    public static boolean isOn(Section s) { return on[s.ordinal()]; }

    /** Call in the init loop. Dpad up/down moves, dpad right toggles. */
    public static void menu(Gamepad now, Gamepad last, Telemetry t) {
        int n = Section.values().length;
        if (now.dpad_down && !last.dpad_down) cursor = (cursor + 1) % n;
        if (now.dpad_up && !last.dpad_up) cursor = (cursor - 1 + n) % n;
        if (now.dpad_right && !last.dpad_right) on[cursor] = !on[cursor];

        t.addLine("=== Telemetry (up/down move, right toggle) ===");
        for (Section s : Section.values()) {
            t.addLine((s.ordinal() == cursor ? "> " : "   ") + (isOn(s) ? "[x] " : "[ ] ") + s);
        }
    }

    /** Call every loop. Prints only the sections you turned on. */
    public static void print(RobotHardware r, Follower f, Telemetry t) {
        if (isOn(Section.LOOP)) {
            t.addLine("--- Loop ---");
            t.addData("Loop (ms)", "%.1f", r.loopTimeMs);
            t.addData("Battery (V)", "%.2f", r.voltage);
        }
        if (isOn(Section.POSE)) {
            t.addLine("--- Pose ---");
            t.addData("x", "%.1f", f.pose().x());
            t.addData("y", "%.1f", f.pose().y());
            t.addData("heading", "%.1f", Math.toDegrees(f.pose().heading()));
        }
        if (isOn(Section.BEAMS)) {
            t.addLine("--- Beams ---");
            t.addData("Front", beam(r.frontBeam));
            t.addData("Mid", beam(r.midBeam));
            t.addData("Pollen", beam(r.pollenBeam));
            t.addData("Nectar", beam(r.nectarBeam));
        }
        if (isOn(Section.INTAKE)) {
            t.addLine("--- Intake ---");
            t.addData("Intake1 pwr / A", "%.2f / %.2f", r.intake1.getPower(), r.amps(r.intake1));
            t.addData("Intake2 pwr / A", "%.2f / %.2f", r.intake2.getPower(), r.amps(r.intake2));
            t.addData("Deploy pos", r.intakeDeploy.getPosition());
        }
        if (isOn(Section.SHOOTER)) {
            t.addLine("--- Shooter ---");
            t.addData("Pollen vel / A", "%.0f / %.2f", r.shooter1.getVelocity(), r.amps(r.shooter1));
            t.addData("Nectar vel / A", "%.0f / %.2f", r.shooter2.getVelocity(), r.amps(r.shooter2));
        }
        if (isOn(Section.TURRET_HOOD)) {
            t.addLine("--- Turret / Hood / Locks ---");
            t.addData("Turret1 / Turret2", "%.3f / %.3f", r.turret1.getPosition(), r.turret2.getPosition());
            t.addData("Hood1 / Hood2", "%.3f / %.3f", r.hood1.getPosition(), r.hood2.getPosition());
            t.addData("Lock1 / Lock2", "%.3f / %.3f", r.feedLock1.getPosition(), r.feedLock2.getPosition());
        }
        if (isOn(Section.COLOR)) {
            t.addLine("--- Color ---");
            t.addData("Distance (cm)", "%.1f", r.colorSensor.getDistance(DistanceUnit.CM));
            t.addData("R / G / B", "%d / %d / %d", r.colorSensor.red(), r.colorSensor.green(), r.colorSensor.blue());
        }
    }

    private static String beam(com.qualcomm.robotcore.hardware.DigitalChannel b) {
        return b.getState() ? "clear" : "BLOCKED";
    }
}