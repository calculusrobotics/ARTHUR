package frc.robot.utils.control.motor;

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
    /** Get the voltage drop across the motor in volts (V) */
    public abstract double getVoltage();

    /** Get the percentage (0 to 1) of the robot's voltage that is seen by the motor controller*/
    public abstract double getPercentVoltage();
}