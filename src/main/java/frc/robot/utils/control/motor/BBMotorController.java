package frc.robot.utils.control.motor;



import frc.robot.utils.data.DataWindow;

import frc.robot.utils.control.controltype.ControlType;
import frc.robot.utils.control.pidf.PID;
import frc.robot.utils.control.motionprofile.motionmagic.MotionMagic;
import frc.robot.utils.control.MotorInfo;

import frc.robot.utils.control.encoder.*;

import frc.robot.utils.math.units.BaseUnit;
import frc.robot.utils.math.units.Unit;
import frc.robot.utils.math.units.Units;
import frc.robot.utils.math.units.BaseUnit.Dimension;
import frc.robot.utils.math.units.UnitBuilder;
import frc.robot.utils.math.units.Quantity;

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



    /** MotionMagic parameters to be used */
    protected MotionMagic motionMagic;



    /**
     * Try to set PID constants of the motor upon addition to pidConstants ArrayList
     * 
     * Will only work for first few (2 for CTRE) slots until there are no more available
     * PID slots in the wrapped motor controller and will have to be changed by the
     * BBMotorController.
     */
    protected abstract void trySetPID(int pidID);

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

        trySetPID(id);

        return id;
    }

    protected int activePIDSlot = 0;
    public int getActivePIDSlot() {
        return activePIDSlot;
    }



    public abstract void setPIDSlot(int pidSlot);





    protected abstract BaseUnit getNativeUnit();
    protected BaseUnit THETA_UNIT_NU; // native encoder position units

    protected abstract BaseUnit getTimeUnit_nu();
    protected final BaseUnit TIME_UNIT_NU = getTimeUnit_nu();
    protected abstract BaseUnit getSecondTimeUnit_nu();
    protected final BaseUnit SECOND_TIME_UNIT_NU = getSecondTimeUnit_nu();

    private Unit OMEGA_UNIT_NU;
    private Unit ALPHA_UNIT_NU;

    protected void updateUnits_nu() {
        THETA_UNIT_NU = getNativeUnit();
        OMEGA_UNIT_NU = (new UnitBuilder()).num(THETA_UNIT_NU).denom(TIME_UNIT_NU).make();
        ALPHA_UNIT_NU = (new UnitBuilder()).num(THETA_UNIT_NU).denom(TIME_UNIT_NU, SECOND_TIME_UNIT_NU).make();
    }





    protected BaseUnit THETA_UNIT_PU = Units.RAD; // preferred unit for input rotations
    protected BaseUnit TIME_UNIT_PU = Units.S;
    protected BaseUnit SECOND_TIME_UNIT_PU = Units.S;

    private enum PositionMeasurement {
        Angle,
        Distance
    }

    private PositionMeasurement positionMeasurement = PositionMeasurement.Angle;

    protected BaseUnit LENGTH_UNIT_PU = Units.FT;
    protected Quantity radius;

    protected Unit OMEGA_UNIT_PU = Units.RAD_PER_S;
    protected Unit ALPHA_UNIT_PU = Units.RAD_PER_S2;
    protected Unit RAD_PER_TIME = Units.RAD_PER_S;
    protected Unit RAD_PER_TIME2 = Units.RAD_PER_S2;
    protected Unit VEL_UNIT_PU = Units.FT_PER_S;
    protected Unit ACC_UNIT_PU = Units.FT_PER_S2;

    public void setRadius(double radius) {
        if (LENGTH_UNIT_PU == null) {
            return;
        }

        this.radius = new Quantity(radius, LENGTH_UNIT_PU);
    }

    public void setRadius(Quantity radius) {
        if (radius.getUnit().isCompatible(Dimension.Length)) {
            this.radius = radius;
        }
    }

    public void setMeasurementToDistance() {
        // needs to have a radius first
        if (radius == null) { return; }

        positionMeasurement = PositionMeasurement.Distance;
    }

    public void setMeasurementToDistance(double radius) {
        // need to have a unit
        if (LENGTH_UNIT_PU == null) {
            return;
        }

        this.radius = new Quantity(radius, LENGTH_UNIT_PU);

        positionMeasurement = PositionMeasurement.Distance;
    }

    public void setMeasurementToDistance(Quantity radius) {
        this.radius = radius;

        positionMeasurement = PositionMeasurement.Distance;
    }

    public void setMeasurementToAngle() {
        positionMeasurement = PositionMeasurement.Angle;
    }

    public void setThetaUnit(BaseUnit thetaUnit) {
        if (thetaUnit.getDimension() != BaseUnit.Dimension.Angle) {
            return;
        }

        THETA_UNIT_PU = thetaUnit;

        updateUnits_pu();
    }

    public void setTimeUnit(BaseUnit timeUnit) {
        if (timeUnit.getDimension() != BaseUnit.Dimension.Time) {
            return;
        }

        TIME_UNIT_PU = timeUnit;

        updateUnits_pu();
    }

    public void setSecondTimeUnit(BaseUnit timeUnit) {
        if (timeUnit.getDimension() != BaseUnit.Dimension.Time) {
            return;
        }

        SECOND_TIME_UNIT_PU = timeUnit;

        updateUnits_pu();
    }

    public void setLengthUnit(BaseUnit lengthUnit) {
        if (lengthUnit.getDimension() != BaseUnit.Dimension.Length) {
            return;
        }

        LENGTH_UNIT_PU = lengthUnit;

        updateUnits_pu();
    }
// haha funny FRC number 254



    private void updateUnits_pu() {
        OMEGA_UNIT_PU = (new UnitBuilder()).num(THETA_UNIT_PU).denom(TIME_UNIT_PU).make();
        ALPHA_UNIT_PU = (new UnitBuilder()).num(THETA_UNIT_PU).denom(SECOND_TIME_UNIT_PU).make();

        RAD_PER_TIME = (new UnitBuilder()).num(Units.RAD).denom(TIME_UNIT_PU).make();
        RAD_PER_TIME2 = (new UnitBuilder()).num(Units.RAD).denom(SECOND_TIME_UNIT_PU).make();
        
        VEL_UNIT_PU = (new UnitBuilder()).num(LENGTH_UNIT_PU).denom(TIME_UNIT_PU).make();
        ACC_UNIT_PU = (new UnitBuilder()).num(LENGTH_UNIT_PU).denom(SECOND_TIME_UNIT_PU).make();
    }





    /**
     * Command the position of the motor to a specified amount of encoder ticks
     * 
     * @param ticks encoder ticks to command the motor to
     * @param controlMethod control method (MotionMagic or PID) to be used
     */
    protected abstract void cmdPosition_nu(double val_nu, ControlType controlMethod);

    public void cmdPosition(double pos, ControlType controlMethod, int pidSlot) {
        if (controlMethod.getVariable() != ControlType.Variable.Position) {
            return;
        }

        setPIDSlot(pidSlot);

        if (positionMeasurement == PositionMeasurement.Angle) {
            Quantity quant = new Quantity(pos, THETA_UNIT_PU);

            cmdPosition_nu(quant.to(THETA_UNIT_NU).getValue(), controlMethod);
        } else if (positionMeasurement == PositionMeasurement.Distance) {
            // l = r theta
            // theta = l / r and also radians are pretty rad
            Quantity quant = new Quantity(pos, LENGTH_UNIT_PU).divide(radius).multiply(Units.RAD);

            cmdPosition_nu(quant.to(THETA_UNIT_NU).getValue(), controlMethod);
        }
    }

    public void cmdPosition(double pos, ControlType controlMethod) {
        cmdPosition(pos, controlMethod, 0);
    }

    public void cmdPosition(Quantity quant, ControlType controlMethod, int pidSlot) {
        if (controlMethod.getVariable() != ControlType.Variable.Position) {
            return;
        }

        setPIDSlot(pidSlot);

        if (quant.getUnit().isCompatible(THETA_UNIT_PU)) {
            cmdPosition_nu(quant.to(THETA_UNIT_NU).getValue(), controlMethod);
        } else if (quant.getUnit().isCompatible(LENGTH_UNIT_PU)) {
            cmdPosition_nu(quant.divide(radius).multiply(Units.RAD).to(THETA_UNIT_NU).getValue(), controlMethod);
        }
    }

    public void cmdPosition(Quantity quant, ControlType controlMethod) {
        cmdPosition(quant, controlMethod, 0);
    }




    /** Type of encoder attached the to motor */
    protected SensorType sensor;

    public void addEncoder(SensorType sensor) {
        this.sensor = sensor;

        if (sensor instanceof QuadratureEncoder) {
            addQuadratureEncoder((QuadratureEncoder) sensor);
        }

        updateUnits_nu();
    }



    protected abstract void addQuadratureEncoder(QuadratureEncoder sensor);



    /**
     * Return the position read on the encoder in ticks
     * 
     * @return position on the encoder in ticks
     */
    public abstract double getPosition_nu();



    /** Configure MotionMagic parameters to use with native units */
    protected abstract void configMotionMagic_nu(double acc, double vel);

    public void configMotionMagic(MotionMagic mm) {
        Quantity vel_pu = new Quantity(mm.getCruiseVelocity(), OMEGA_UNIT_PU);
        Quantity acc_pu = new Quantity(mm.getAcceleration(), ALPHA_UNIT_PU);

        configMotionMagic_nu(acc_pu.to(ALPHA_UNIT_NU).getValue(), vel_pu.to(OMEGA_UNIT_NU).getValue());

        motionMagic = mm;
    }



    /**
     * Return the position read on the encoder in preferred units
     * 
     * @return position on the encoder in preferred units
     */
    public double getPosition() {
        Quantity theta_nu = new Quantity(getPosition_nu(), THETA_UNIT_NU);

        if (positionMeasurement == PositionMeasurement.Angle) {
            return theta_nu.to(THETA_UNIT_PU).getValue();
        } else {
            // l = r theta but theta in rads that don't really exist but oh well
            return theta_nu.to(Units.RAD).divide(Units.RAD).multiply(radius).getValue();
        }
    }

    public double getPosition(BaseUnit unit) {
        Quantity theta_nu = new Quantity(getPosition_nu(), THETA_UNIT_NU);

        if (unit.getDimension() == Dimension.Length) {
            if (radius != null) {
                return theta_nu.to(THETA_UNIT_PU).divide(Units.RAD).multiply(radius).to(unit).getValue();
            } else {
                return 0; // rip
            }
        } else if (unit.getDimension() == Dimension.Angle) {
            return theta_nu.to(unit).getValue();
        } else {
            return 0; // rip
        }
    }



    public enum PositionControl {
        Position,
        MotionMagic // TODO: SmartMotion?
    }



    /**
     * Get the velocity in native encoder units
     * 
     * @return velocity in native encoder units
     */
    public abstract double getVelocity_nu();

    /**
     * Get the velocity of the object in preferred units of the object
     */
    public double getVelocity() {
        // ticks per TIME_PERIOD
        Quantity vel_nu = new Quantity(getVelocity_nu(), OMEGA_UNIT_NU);

        if (positionMeasurement == PositionMeasurement.Angle) {
            return vel_nu.to(OMEGA_UNIT_PU).getValue();
        } else {
            // v = r omega but omega in rads that don't really exist but oh well
            return vel_nu.to(RAD_PER_TIME).divide(Units.RAD).multiply(radius).getValue();
        }
    }



    /**
     * Set the position in revolutions (OF ENCODER - not necessarily of thing being controller). 
     */
    //public abstract void cmdPosition(double revs);


    
    /** Get the voltage drop across the motor in volts (V) */
    public abstract double getVoltage();

    /** Get the percentage (0 to 1) of the robot's voltage that is seen by the motor controller*/
    public abstract double getPercentVoltage();



    public abstract void follow(BBMotorController motorController);










    protected boolean hasDiagnostics;
    protected DataWindow<MotorInfo> diagnosticData;
}