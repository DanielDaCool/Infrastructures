package frc.robot.Path;

import edu.wpi.first.math.geometry.Translation2d;

/** Small angle and vector helpers shared by the Path classes (design doc section 3, 6.3). */
final class PathMath {

    /** A change within this of pi turns CCW (section 10, half-turn tie). */
    static final double HALF_TURN_TIE = 1e-9;

    private PathMath() {
    }

    /** @return angle wrapped to (-pi, pi] */
    static double wrap(double angle) {
        double a = mod2pi(angle + Math.PI) - Math.PI;
        return a <= -Math.PI ? a + 2.0 * Math.PI : a;
    }

    /** @return angle wrapped to [0, 2pi) */
    static double mod2pi(double angle) {
        double twoPi = 2.0 * Math.PI;
        double a = angle % twoPi;
        if (a < 0) {
            a += twoPi;
        }
        return a >= twoPi ? 0.0 : a;   // a tiny negative angle rounds to 2pi after the add
    }

    /** @return v rotated by +90 degrees: (x, y) becomes (-y, x) */
    static Translation2d rot90(Translation2d v) {
        return new Translation2d(-v.getY(), v.getX());
    }

    /** @return z component of a x b */
    static double cross(double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    }

    /** @return sqrt(max(0, x)) */
    static double vsqrt(double x) {
        return Math.sqrt(Math.max(0.0, x));
    }

    /**
     * The short way around (section 6.3): wrapped to (-pi, pi], and a change within
     * {@link #HALF_TURN_TIE} of a half turn is +pi (CCW).
     */
    static double shortTurn(double d) {
        double w = wrap(d);
        if (Math.abs(w) > Math.PI - HALF_TURN_TIE) {
            return Math.PI;
        }
        return w;
    }
}
