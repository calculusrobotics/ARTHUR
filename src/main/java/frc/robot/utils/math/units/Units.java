package frc.robot.utils.math.units;

import frc.robot.utils.math.units.BaseUnit.Dimension;

public class Units {
    // Length units
    public static final BaseUnit IN = new BaseUnit(Dimension.Length, 1);
    public static final BaseUnit FT = new BaseUnit(IN, 1.0/12);
    public static final BaseUnit CM = new BaseUnit(IN, 2.54);
    public static final BaseUnit M =  new BaseUnit(CM, 0.01);
    public static final BaseUnit YARD    = new BaseUnit(FT, 1.0/3);
    public static final BaseUnit FURLONG = new BaseUnit(YARD, 1.0/220);
    public static final BaseUnit SMOOT = new BaseUnit(FT, 1.0/(5 + 7.0/12.0)); // 5 foot 7 per Smoot

    // Angles
    public static final BaseUnit RAD = new BaseUnit(Dimension.Angle, 1);
    public static final BaseUnit REV = new BaseUnit(RAD, 1.0/(2 * Math.PI));
    public static final BaseUnit DEG = new BaseUnit(REV, 360);

    // Mass
    public static final BaseUnit LB = new BaseUnit(Dimension.Mass, 1);
    public static final BaseUnit KG = new BaseUnit(LB, 0.453592);

    // Time
    public static final BaseUnit S   = new BaseUnit(Dimension.Time, 1);
    public static final BaseUnit MIN = new BaseUnit(S, 1.0/60);
    public static final BaseUnit MS  = new BaseUnit(S, 1000);
    public static final BaseUnit MS100 = new BaseUnit(MS, 1.0/100);
    public static final BaseUnit HR   = new BaseUnit(MIN, 1.0/60);
    public static final BaseUnit DAY   = new BaseUnit(HR, 1.0/12);
    public static final BaseUnit WEEK   = new BaseUnit(DAY, 1.0/7);
    public static final BaseUnit FORTNIGHT   = new BaseUnit(DAY, 1/2.0);

    // Velocity
    public static final Unit IN_PER_S = (new UnitBuilder()).num(IN).denom(S).make();
    public static final Unit FT_PER_S = (new UnitBuilder()).num(FT).denom(S).make();
    public static final Unit M_PER_S  = (new UnitBuilder()).num(M).denom(S).make();
    public static final Unit FURLONGS_PER_FORTNIGHT = (new UnitBuilder()).num(FURLONG).denom(FORTNIGHT).make();

    // Acceleration
    public static final Unit IN_PER_S2 = (new UnitBuilder()).num(IN).denom(S, S).make();
    public static final Unit FT_PER_S2 = (new UnitBuilder()).num(FT).denom(S, S).make();
    public static final Unit M_PER_S2  = (new UnitBuilder()).num(M).denom(S, S).make();
    public static final Unit G         = (new UnitBuilder()).num(M).denom(S, S).coeff(9.80665).make();

    // Angular velocity
    public static final Unit RAD_PER_S = (new UnitBuilder()).num(RAD).denom(S).make();
    public static final Unit DEG_PER_S = (new UnitBuilder()).num(DEG).denom(S).make();
    public static final Unit REV_PER_S = (new UnitBuilder()).num(REV).denom(S).make();
    public static final Unit RPM       = (new UnitBuilder()).num(REV).denom(MIN).make();

    // Angular acceleration
    public static final Unit RAD_PER_S2 = (new UnitBuilder()).num(RAD).denom(S, S).make();
    public static final Unit DEG_PER_S2 = (new UnitBuilder()).num(DEG).denom(S, S).make();
    public static final Unit REV_PER_S2 = (new UnitBuilder()).num(REV).denom(S, S).make();
    public static final Unit REV_PER_MIN_S       = (new UnitBuilder()).num(REV).denom(MIN, S).make();
    public static final Unit REV_PER_MIN2       = (new UnitBuilder()).num(REV).denom(MIN, MIN).make();

    // Momentum
    public static final Unit GB = (new UnitBuilder()).num(KG, M).denom(S).make(); // 1 greenberg = 1 kgm/s
}