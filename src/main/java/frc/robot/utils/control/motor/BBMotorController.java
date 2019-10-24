package frc.robot.utils.control.motor;



import frc.robot.utils.control.pidf.PID;

import frc.robot.utils.control.encoder.*;

import java.util.ArrayList;
import java.util.HashMap;



/**
 * To generalize to different types of motor controllers (TalonSRX/SparkMax as of right now)
 * that all have different method names/etc, it's useful to have an abstract class for a generic
 * motor controller (Bit Bucket Motor Controller). It's annoying enough to memorize the
 * oddly specific names some APIs (*cough* CTRE *cough*) gives to their methods so a convenient
 * way to circumvent this is a standardized set of methods for all motor controllers, intuitively
 * named
 * 
 * We use an abstract class instead of an interface because it allows us to have standardized
 * methods for all sorts of motor controllers that are not provided/may not be "standard"
 * in motor controller APIs we use. The subclasses themselves don't need to extend their
 * respective motor controller since they can just store an instance of such.
 * 
 * We can implement any methods in subclasses such as BBSparkMax and BBTalonSRX.
 * These wrapper classes contain their corresponding motor controller and by use of
 * standardized methods in BBMotorController can give other classes information about
 * or let other classes control the inside motor.
 * 
 * Just makes the code nicer to write and shorter (CTRE is very verbose) and hopefully
 * easier to understand, which is always good.
 * 
 * Generalizing also allows us to generalize in our control methods. Do we want to use
 * provided functional PID(F) or do we want to experiment with state-space?
 */
public abstract class BBMotorController {
    /*
     * NOTE: traditionally, you put all fields and enums before all methods.
     * This will not be the case in this class - it is written in a way that makes sense
     * as you read from top to bottom. Fields will be declared as needed more the methods
     * that follow
     */


    
    public void addEncoder(SensorType sensor) {
        if (sensor instanceof QuadratureEncoder) {
            addQuadraticEncoder((QuadratureEncoder) sensor);
        }
    }



    protected abstract void addQuadraticEncoder(QuadratureEncoder sensor);

    

    /*
     * While TalonSRX reserves 2 slots for PIDF constants (0 and 1), SparkMax does not
     * so we cannot generally assume this functionality will be provided and should thus
     * implement it ourselves.
     */
    /**
     * ArrayList of PID constants that are stored in our motor controller wrapper objects.
     * Internally, TalonSRXs can store at most 2, but we can store more in motor controller
     * objects in our code so long as we can efficiently communicate when to switch to what set of
     * constants to the provided CTRE/RevRobotics/... motor controller objects.
     */
    protected ArrayList<PID> pidConstants;
    /**
     * To facilitate keeping track of which PID(F) constants correspond to what "slot" in
     * our wrapper class (which then loosely corresponds to a slot in the TalonSRX if using
     * those), we can give names to each slot.
     * 
     * If no name is provided, will default to "PID[NUMBER]" where [NUMBER] is the number
     * of the slot as its added (first -> 0, second -> 1, ...)
     */
    protected HashMap<String, Integer> pidNames;

    /**
     * Add a set of PID constants to the motor controller's stored set
     * 
     * @param constants PID(F) constants to add to set of internally stored constants
     * 
     * @return numeric ID of these constants
     */
    public int addPID(PID constants) {
        int id = pidConstants.size();
        String name = "PID" + id;

        return addPID(constants, name);
    }

    /**
     * Add a set of PID constants to the motor controller's stored set,
     * indexed by a name
     * 
     * @param constants PID(F) constants to add to set of internally stored constants
     * @param name name of these PID(F) constants
     * 
     * @return numeric ID of these constants
     */
    public int addPID(PID constants, String name) {
        int id = pidConstants.size();

        pidConstants.add(constants);
        // name corresponds to id-th set in pidConstants
        pidNames.put(name, id);

        return id;
    }

    protected int activePIDSlot = 0;
    public int getActivePIDSlot() {
        return activePIDSlot;
    }

    public enum PositionControl {
        Position,
        MotionMagic // TODO: SmartMotion?
    }

    /**
     * Set the position in revolutions (OF ENCODER - not necessarily of thing being controller). 
     */
    //public abstract void setPosition(double revs);

    /** Get the voltage drop across the motor in volts (V) */
    public abstract double getVoltage();

    /** Get the percentage (0 to 1) of the robot's voltage that is seen by the motor controller*/
    public abstract double getPercentVoltage();
}