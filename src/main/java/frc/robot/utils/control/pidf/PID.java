package frc.robot.utils.control.pidf;



import frc.robot.utils.control.controltype.ControlType;



/**
 * Set of constants for a PID (proportional-integral-derivative) controller.
 * Typically used solely for position control (though MotionMagic is smoother)
 */
public class PID {
    protected final ControlType CONTROL_TYPE;



    // PID constants
    // these are protected so subclasses such as PIDF can access them easily
    protected final double KP;
    protected final double KI;
    protected final double KD;

    protected double iZone;

    private int uses = 0;

    

    /**
     * Generate a set of PID constants
     * 
     * @param controlType type of control these PID constants are intended for
     * @param kP proportional constant
     * @param kI integral constant
     * @param kD derivative constant
     */
    public PID(ControlType controlType, double kP, double kI, double kD) {
        CONTROL_TYPE = controlType;

        KP = kP;
        KI = kI;
        KD = kD;
    }

    /**
     * Generate a set of PID constants
     * 
     * @param controlType type of control these PID constants are intended for
     * @param kP proportional constant
     * @param kI integral constant
     * @param kD derivative constant
     * @param iZone integral zone
     */
    public PID(ControlType controlType, double kP, double kI, double kD, double iZone) {
        this(controlType, kP, kI, kD);

        this.iZone = iZone;
    }



    public double getKP() { return KP; }
    public double getKI() { return KI; }
    public double getKD() { return KD; }
    public double getIZone() { return iZone; }

    public ControlType getControlType() { return CONTROL_TYPE; }

    public void use() {
        uses++;
    }

    public int getUses() {
        return uses;
    }
}