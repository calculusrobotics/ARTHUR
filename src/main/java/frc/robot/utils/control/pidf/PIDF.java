package frc.robot.utils.control.pidf;



import frc.robot.utils.control.ControlType;



public class PIDF extends PID {
    protected final double KF;


    
    /**
     * Generate a set of PID constants
     * 
     * @param controlType type of control these PID constants are intended for
     * @param kP proportional constant
     * @param kI integral constant
     * @param kD derivative constant
     * @param kF feed-forward constant
     */
    public PIDF(ControlType controlType, double kP, double kI, double kD, double kF) {
        super(controlType, kP, kI, kD);
        
        KF = kF;
    }

    /**
     * Generate a set of PID constants
     * 
     * @param controlType type of control these PID constants are intended for
     * @param kP proportional constant
     * @param kI integral constant
     * @param kD derivative constant
     * @param kF feed-forward constant
     * @param iZone integral zone
     */
    public PIDF(ControlType controlType, double kP, double kI, double kD, double kF, double iZone) {
        this(controlType, kP, kI, kD, kF);

        this.iZone = iZone;
    }

    

    public double getKF() { return KF; }
}