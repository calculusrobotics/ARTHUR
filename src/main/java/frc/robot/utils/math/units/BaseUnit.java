package frc.robot.utils.math.units;

import java.util.HashMap;

/**
 * Class for units of elementary dimensions.
 * Instead of converting between units directly, units are first converted to some "standard"
 * and then converted from the standard to the other unit. If every unit knows its conversion
 * to the standard, you can convert to standard units and then convert from standard to
 * others. Standard can be whatever you want so long as you are consistent.
 */
public class BaseUnit {
    /** Dimensions for BaseUnits to measure */
    public enum Dimension {
        Length,
        Angle,
        Mass,
        Time
    }





    /** how many of this unit in 1 standard unit */
    private double perStandard;
    /** Dimension this unit measures */
    private final Dimension DIM;

    /** Whether to store conversions to other units. Trade-off between computation time and memory space. */
    private boolean storeConversions = false;
    /** Stored conversions from a BaseUnit of the same Dimension to this unit/other unit */
    private HashMap<BaseUnit, Double> conversions = new HashMap<BaseUnit, Double>();


    /**
     * Create a BaseUnit
     * 
     * @param dim dimension the unit measures
     */
    public BaseUnit(Dimension dim) {
        this.DIM = dim;
    }

    /**
     * Create a BaseUnit
     * 
     * @param dim dimension the unit measures
     * @param perStandard how many of this unit are there per standard unit
     */
    public BaseUnit(Dimension dim, double perStandard) {
        this(dim);

        this.perStandard = perStandard;
    }

    /**
     * Create a BaseUnit based off of conversion to another
     * 
     * @param unit other unit that conversion is specified to
     * @param per how many of this BaseUnit to other unit
     */
    public BaseUnit(BaseUnit unit, double per) {
        this(unit.getDimension());

        setPer(unit, per);
    }

    

    /**
     * Indicate to store conversions. If you are converting to the
     * same set of units frequently, this is a trade-off between
     * computation time for every conversion and memory.
     * 
     * Instead of converting to standard and then to other unit,
     * this would allow the Unit to convert directly because
     * it knows the conversion to the stored units.
     */
    public void storeConversions() {
        storeConversions = true;
    }


    
    /**
     * Return the dimension this unit measures
     * 
     * @return dimension unit measures
     */
    public Dimension getDimension() {
        return DIM;
    }

    /**
     * Sets how many of this unit are in a standard unit
     * 
     * @param perStandard how many of this unit are in the corresponding standard
     */
    public void setPerStandard(double perStandard) {
        this.perStandard = perStandard;
    }

    /**
     * Get the amount of this unit in the corresponding standard
     */
    public double perStandard() {
        return perStandard;
    }

    /**
     * Set conversions based on conversion to another unit
     * 
     * @param bu other unit to base conversions off of
     * @param per amount of this unit per 1 bu
     */
    public void setPer(BaseUnit bu, double per) {
        // make sure dimensions are compatible
        if (bu.getDimension() != DIM) {
            return;
        }

        // if storing conversions, might as well store this given one
        if (storeConversions) {
            conversions.put(bu, per);
        }

        // units / 1 bu * bu/std = units/std
        setPerStandard(per * bu.perStandard());
    }

    /**
     * Determine how many of this unit are in another of the same dimension
     * 
     * @param bu other BaseUnit to determine conversion to
     * @return how many of this unit are in bu, or 0 if dimensions are incompatible
     */
    public double per(BaseUnit bu) {
        // don't double calculate if you already know the conversion
        if (storeConversions) {
            Double conv = conversions.get(bu);

            // if you know the conversion, return it
            if (conv != null) {
                return conv;
            }
        }

        // return 0 if dimensions are incompatible
        // not needed at start because only stores comptatible BaseUnit conversions
        if (bu.getDimension() != DIM) {
            return 0; // ig
        }

        // units/std * std/units2 = units/units2
        double per = perStandard / bu.perStandard();
        // if storing conversions, store this one now that the unit knows it
        if (storeConversions) {
            conversions.put(bu, per);
        }

        return per;
    }

    /**
     * Convert amounts of this unit to another
     * 
     * @param bu unit to convert to
     * @param num amount of this unit
     * 
     * @return num in units of bu or 0 if unit dimensions are incompatible
     */
    public double to(BaseUnit bu, double num) {
        if (bu.getDimension() != DIM) {
            return 0; // ig
        }

        // units * units2/unit1
        return num / per(bu);
    }
}