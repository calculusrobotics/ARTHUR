package frc.robot.utils.math.units;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;
import java.util.HashMap;

import frc.robot.utils.math.units.BaseUnit.Dimension;



/**
 * Unit formed by multiplying/dividing BaseUnits
 */
public class CompositeUnit {
    /*
     * instead of storing numerator units and denominator units, we can store
     * numerator and denominator units PER dimension. This allows us to group
     * units into dimensions. The reason this is helpful is as follows:
     * 
     * Suppose you are looking at units of momentum and have
     * kgm/s: KG * M / S
     * lbft/s: FT * LB / S
     * Simply looping through these to determine the conversion would require
     * dividing a kilogram by a foot and a meter by a pound. Basically grouping
     * would let us divide the correct units by each other to get the conversion.
     * aka invariant under switching order of inputed units.
     * 
     * Note: units are reduced in constructor so that there are none of the
     * same unit in both the numerator and denominator
     */
    /** BaseUnits in numerator grouped by dimension they measure */
    private final HashMap<Dimension, ArrayList<BaseUnit>> NUMERATOR;
    /** BaseUnits in denominator grouped by dimension they measure */
    private final HashMap<Dimension, ArrayList<BaseUnit>> DENOMINATOR;
    /**
     * Total count of how many of each dimension are in the unit.
     * For example, kg m^2 -> Mass: 1, Length: 2
     */
    private final HashMap<Dimension, Integer> DIMENSIONS;
    /**
     * Coefficient in front of unit resulting from cancellation within
     * same dimension. For example, if unit is a kg inch/cm then it is just
     * 2.54 kg
     */
    private final double COEFF;



    /** Whether to store conversions to other units. Trade-off between computation time and memory space. */
    private boolean storeConversions = false;
    /** Stored conversions from a CompositeUnit of the same Dimension to this unit/other unit */
    private HashMap<CompositeUnit, Double> conversions = new HashMap<CompositeUnit, Double>();



    /**
     * Warning: this constructor may modify numerator and denominator variables
     * 
     * Create a CompositeUnit with a list of numerator and denominator units
     * 
     * @param numerator list of numerator units
     * @param denominator list of denominator units
     */
    public CompositeUnit(List<BaseUnit> numerator, List<BaseUnit> denominator) {
        HashMap<Dimension, ArrayList<BaseUnit>> nums   = new HashMap<Dimension, ArrayList<BaseUnit>>();
        HashMap<Dimension, ArrayList<BaseUnit>> denoms = new HashMap<Dimension, ArrayList<BaseUnit>>();
        HashMap<Dimension, Integer> dims = new HashMap<Dimension, Integer>();
        double coeff = 1;

        Dimension[] dimList = Dimension.values();

        for (int i = 0; i < dimList.length; i++) {
            Dimension dim = dimList[i];

            nums.put(dim, new ArrayList<BaseUnit>());
            denoms.put(dim, new ArrayList<BaseUnit>());
            dims.put(dim, 0);
        }



        for (int i = 0; i < numerator.size(); i++) {
            BaseUnit num = numerator.get(i);

            for (int j = 0; j < denominator.size(); j++) {
                BaseUnit denom = denominator.get(j);

                if (num.getDimension() == denom.getDimension()) {
                    // num / denom = num.per(denom)
                    coeff *= num.per(denom);

                    numerator.remove(i);
                    denominator.remove(j);

                    i--;
                }
            }
        }

        for (int i = 0; i < numerator.size(); i++) {
            BaseUnit num = numerator.get(i);
            Dimension dim = num.getDimension();

            nums.get(dim).add(num);
            dims.put(dim, dims.get(dim) + 1);
        }

        for (int i = 0; i < denominator.size(); i++) {
            BaseUnit denom = denominator.get(i);
            Dimension dim = denom.getDimension();

            denoms.get(dim).add(denom);
            dims.put(dim, dims.get(dim) - 1);
        }

        NUMERATOR = nums;
        DENOMINATOR = denoms;
        DIMENSIONS = dims;
        COEFF = coeff;
    }

    public CompositeUnit(BaseUnit[] numerator, BaseUnit[] denominator) {
        this(Arrays.asList(numerator), Arrays.asList(denominator));
    }



    public double getCoefficient() {
        return COEFF;
    }

    public ArrayList<BaseUnit> getNumeratorTerms(Dimension dim) {
        return NUMERATOR.get(dim);
    }

    public ArrayList<BaseUnit> getDenominatorTerms(Dimension dim) {
        return DENOMINATOR.get(dim);
    }

    public int getDimension(Dimension dim) {
        return DIMENSIONS.get(dim);
    }

    public boolean isCompatible(CompositeUnit unit2) {
        Dimension[] dimList = Dimension.values();

        for (int i = 0; i < dimList.length; i++) {
            Dimension dim = dimList[i];

            if (getDimension(dim) != unit2.getDimension(dim)) {
                return false;
            }
        }

        return true;
    }

    public double per(CompositeUnit unit2) {
        if (storeConversions) {
            Double conv = conversions.get(unit2);

            if (conv != null) {
                return conv;
            }
        }

        if (!isCompatible(unit2)) {
            return 0; // ig
        }

        // units = coeff * nums / denoms
        // units2 = coeff2 * nums2 / denoms2
        // units/unit2 = coeff/coeff2 * nums/nums2 * denoms2/denoms

        double per = COEFF / unit2.getCoefficient();

        Dimension[] dimList = Dimension.values();

        for (int i = 0; i < dimList.length; i++) {
            Dimension dim = dimList[i];

            ArrayList<BaseUnit> numTerms = getNumeratorTerms(dim);
            ArrayList<BaseUnit> numTerms2 = unit2.getNumeratorTerms(dim);

            for (int j = 0; j < numTerms.size(); j++) {
                per *= numTerms.get(j).per(numTerms2.get(j));
            }

            ArrayList<BaseUnit> denomTerms = getNumeratorTerms(dim);
            ArrayList<BaseUnit> denomTerms2 = unit2.getNumeratorTerms(dim);

            for (int j = 0; j < denomTerms.size(); j++) {
                per *= denomTerms2.get(j).per(denomTerms.get(j));
            }
        }

        if (storeConversions) {
            conversions.put(unit2, per);
        }

        return per;
    }

    public double to(CompositeUnit unit2, double num) {
        if (!isCompatible(unit2)) {
            return 0; // ig
        }

        // units * units2/unit1
        return num / per(unit2);
    }
}