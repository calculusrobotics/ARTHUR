package frc.robot.utils.control.motionprofile.motionmagic;

public class MotionMagic {
    private final double ACC;
    private final double CRUISE_VELOCITY;

    public MotionMagic(double acc, double cruiseVelocity) {
        ACC = acc;
        CRUISE_VELOCITY = cruiseVelocity;
    }

    public double getAcceleration() {
        return ACC;
    }

    public double getCruiseVelocity() {
        return CRUISE_VELOCITY;
    }
}