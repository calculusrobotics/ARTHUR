package frc.robot.utils.control.statespace;

import frc.robot.utils.math.units.Quantity;
import frc.robot.utils.math.units.Units;

public enum Motors {
	BAG       (1.0684221022699664E-5, 0.22805452519362293, 0.00819095836320447,   0.008398404204492988),
	CIM       (8.906757474046268E-5,  0.09156461027640256, 0.018412117049746626,  0.021056415791085924),
	Falcon500 (4.093255526065792E-5,  0.04666464678901319, 0.01823032208047652,   0.017857232059815025),
	MiniCIM   (7.696547029161103E-5,  0.13424159295451685, 0.015762200360164944,  0.01896789440121476),
	NEO       (4.355028176267936E-5,  0.07225128353091055, 0.02020627566152965,   0.01934918759361202),
	pro775    (1.8880242321090976E-6, 0.08981161402542714, 0.0052988851945534425, 0.006084753007054214);



    private final Quantity B;
    private final Quantity R;
    private final Quantity K_T;
    private final Quantity K_W;

    private Motors(double b, double R, double Kt, double Kw) {
        B = new Quantity(b, Units.Nm.divide(Units.RAD_PER_S));
        this.R = new Quantity(b, Units.Ohm);
        K_T = new Quantity(Kt, Units.Nm.divide(Units.A));
        K_W = new Quantity(Kw, Units.V.divide(Units.RAD_PER_S));
    }

    

    public Quantity getKF() {
        return (B.multiply(R).divide(K_T)).add(K_W);
    }
}