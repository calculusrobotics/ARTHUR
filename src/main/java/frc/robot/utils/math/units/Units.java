package frc.robot.utils.math.units;

import frc.robot.utils.math.units.BaseUnit.Dimension;

public class Units {
    // Length units
    public static final BaseUnit IN = new BaseUnit(Dimension.Length, 1);
    public static final BaseUnit FT = new BaseUnit(IN, 1.0/12);
    public static final BaseUnit CM = new BaseUnit(IN, 2.54);
    public static final BaseUnit M = new BaseUnit(CM, 0.01);

    // Angles
    public static final BaseUnit RAD = new BaseUnit(Dimension.Angle, 1);
    public static final BaseUnit REV = new BaseUnit(RAD, 1.0/(2 * Math.PI));
    public static final BaseUnit DEG = new BaseUnit(REV, 360);

    // Mass
    public static final BaseUnit LB = new BaseUnit(Dimension.Mass, 1);
    public static final BaseUnit KG = new BaseUnit(LB, 0.453592);

    // Time
    public static final BaseUnit S = new BaseUnit(Dimension.Time, 1);
    public static final BaseUnit MIN = new BaseUnit(S, 1.0/60);
    public static final BaseUnit MS = new BaseUnit(S, 1000);

    // Velocity
    public static final CompositeUnit IN_PER_S = (new UnitBuilder()).num(IN).denom(S).make();
    public static final CompositeUnit FT_PER_S = (new UnitBuilder()).num(FT).denom(S).make();
    public static final CompositeUnit M_PER_S = (new UnitBuilder()).num(M).denom(S).make();

    // Acceleration
    public static final CompositeUnit IN_PER_S2 = (new UnitBuilder()).num(IN).denom(S, S).make();
    public static final CompositeUnit FT_PER_S2 = (new UnitBuilder()).num(FT).denom(S, S).make();
    public static final CompositeUnit M_PER_S2 = (new UnitBuilder()).num(M).denom(S, S).make();

    // Angular velocity
    public static final CompositeUnit RAD_PER_S = (new UnitBuilder()).num(RAD).denom(S).make();
    public static final CompositeUnit REV_PER_S = (new UnitBuilder()).num(REV).denom(S).make();
    public static final CompositeUnit RPM = (new UnitBuilder()).num(REV).denom(MIN).make();
}