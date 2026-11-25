package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Flywheel velocity controller: kS + kV feedforward, kP + kD feedback,
 * voltage compensation and a "boost" for fast recovery after a shot.
 *
 *   power = [ kS*sign(target) + kV*target + kP*error - kD*dVel/dt ] * (nominalV / batteryV)
 *
 *   kS  power needed to just overcome friction            (power)
 *   kV  power per ticks/s, holds speed with zero error     (power per ticks/s)
 *   kP  power per ticks/s of error                         (power per ticks/s)
 *   kD  damping on the measured velocity change (D on measurement, so no
 *       kick when the target changes)                     (power per ticks/s^2)
 *
 * BOOST: when velocity falls more than boostEnter below target (a ball just
 * went through), output boostPower (e.g. full power) until the error is back
 * under boostExit. Hysteresis stops it chattering. This is what recovers RPM
 * between shots much faster than kP alone.
 *
 * The REV hub's built-in "F" is the same idea as kV here, only in other units.
 */
public class VelocityFFController {

    // Read-only after update(): for telemetry / graphs
    public double error, ffTerm, ksTerm, pTerm, dTerm, raw, output;
    public boolean boosting = false;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastMeasured = 0;
    private double filtDeriv = 0;
    private boolean first = true;

    /** Call when the shooter is switched off / mode changes. */
    public void reset() {
        first = true;
        boosting = false;
        filtDeriv = 0;
        error = ffTerm = ksTerm = pTerm = dTerm = raw = output = 0;
    }

    /**
     * @param target       ticks/s (0 = off)
     * @param measured     ticks/s from getVelocity()
     * @param dFilter      0..1, low-pass on the derivative (1 = no filter)
     * @param voltageScale nominalV / batteryV (1 = no compensation)
     * @return motor power 0..1
     */
    public double update(double target, double measured,
                         double kS, double kV, double kP, double kD, double dFilter,
                         boolean boostEnabled, double boostEnter, double boostExit, double boostPower,
                         double voltageScale) {
        double dt = timer.seconds();
        timer.reset();

        if (target <= 0) {
            reset();
            lastMeasured = measured;
            return 0;
        }

        error = target - measured;

        double deriv = 0;
        if (!first && dt > 1e-4) deriv = (measured - lastMeasured) / dt;
        first = false;
        lastMeasured = measured;
        double a = Range.clip(dFilter, 0, 1);
        filtDeriv += a * (deriv - filtDeriv);

        ksTerm = kS * Math.signum(target);
        ffTerm = kV * target;
        pTerm = kP * error;
        dTerm = -kD * filtDeriv;
        raw = ksTerm + ffTerm + pTerm + dTerm;

        // Boost with hysteresis
        if (boostEnabled) {
            if (error > boostEnter) boosting = true;
            else if (error < boostExit) boosting = false;
        } else {
            boosting = false;
        }

        output = boosting
                ? Range.clip(boostPower, 0, 1)
                : Range.clip(raw * voltageScale, 0, 1);
        return output;
    }
}
