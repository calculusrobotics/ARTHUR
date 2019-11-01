package frc.robot.utils.math.units;

public class UnitQuantity {
    private final double VALUE;
    private final Unit UNIT;



    public UnitQuantity(double value, Unit unit) {
        VALUE = value;
        UNIT = unit;
    }



    public Unit getUnit() { return UNIT; }
    public double getValue() { return VALUE; }

    public UnitQuantity convert(Unit unit2) {
        return new UnitQuantity(VALUE / UNIT.per(unit2), unit2);
    }

    public UnitQuantity multiply(UnitQuantity q2) {
        return new UnitQuantity(VALUE * q2.getValue(), UNIT.multiply(q2.getUnit()));
    }

    public UnitQuantity divide(UnitQuantity q2) {
        return new UnitQuantity(VALUE / q2.getValue(), UNIT.divide(q2.getUnit()));
    }
}