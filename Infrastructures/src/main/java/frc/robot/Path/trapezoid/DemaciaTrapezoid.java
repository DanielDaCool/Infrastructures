package frc.robot.Path.trapezoid;

public class DemaciaTrapezoid {
    private final double maxVelocity;
    private final double maxAccel;

    public DemaciaTrapezoid(double maxVelocity, double maxAccel) {
        this.maxAccel = maxAccel;
        this.maxVelocity = maxVelocity;
    }

    public double nextVelocity(double distance, double currentVelocity, double endVelocity){
        return nextVelocity(distance, currentVelocity, endVelocity, maxVelocity);
    }

    public double nextVelocity(double distance, double currentVelocity, double endVelocity, double maxVelocity){

        double vmax = Math.sqrt((distance * 2 * maxAccel + currentVelocity*currentVelocity + endVelocity*endVelocity) / 2);
        if(vmax > currentVelocity) { // we can accelerate
            return Math.min(currentVelocity + maxAccel * 0.02, maxVelocity);
        } else { // we need to decelerate
            double decelTime = 2 * distance / (currentVelocity + endVelocity);
            if(currentVelocity + endVelocity <= 0 || decelTime < 0.08) { // first check avoids 0/0
                return endVelocity;
            } else {
                return currentVelocity - (currentVelocity - endVelocity) * 0.02 / decelTime;
            }
        }
    }
}
