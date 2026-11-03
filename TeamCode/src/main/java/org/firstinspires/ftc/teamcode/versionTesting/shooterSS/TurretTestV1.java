package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp
@Config
public class TurretTestV1 extends LinearOpMode {
    public static double turret1Pos = Globals.turretInit;
    public static double turret2Pos = Globals.turretInit;
    public static double step = 0.0001;
    public static boolean pollen = true;
    boolean lastRB = false;
    boolean lastLB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo turret1 = hardwareMap.get(Servo.class, "turret1");
        Servo turret2 = hardwareMap.get(Servo.class, "turret2");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double adjustment = 0;
            if (gamepad1.right_bumper && !lastRB) adjustment += step;
            if (gamepad1.left_bumper && !lastLB) adjustment -= step;
            lastRB = gamepad1.right_bumper;
            lastLB = gamepad1.left_bumper;
            if (pollen) turret1Pos = Range.clip(turret1Pos + adjustment, 0, 1);
            else turret2Pos = Range.clip(turret2Pos + adjustment, 0, 1);
            turret1.setPosition(turret1Pos);
            turret2.setPosition(turret2Pos);
            telemetry.addData("Bumper control", pollen ? "Pollen" : "Nectar");
            telemetry.addData("turret1 pos", turret1.getPosition());
            telemetry.addData("turret2 pos", turret2.getPosition());
            telemetry.update();
        }
    }
}
