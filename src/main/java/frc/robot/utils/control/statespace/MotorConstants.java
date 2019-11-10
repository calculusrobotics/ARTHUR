package frc.robot.utils.control.statespace;

import frc.robot.utils.math.units.Quantity;
import frc.robot.utils.math.units.Units;

public enum MotorConstants {
    Falcon500 (0.0000409325552606525,    0.04666464688065873, 0.0182303220804765,   0.017857232042234553),
    Neo       (0.00004355028176268708,   0.07225128326877166, 0.020206275661529688, 0.01934918762903231),
    CIM       (0.0000890675747404633,    0.09156461027633264, 0.018412117049746636, 0.021056415791094345),
    MiniCIM   (0.00007696547029161162,   0.1342415885191136,  0.01576220036016496,  0.018967894742216666),
    BAG       (0.000010684221022699957,  0.22805452601252257, 0.008190958363204482, 0.008398404188045239),
    pro775    (0.0000018880242321093018, 0.089811613777675,   0.005298885194553443, 0.006084753015453335);

    private final Quantity B;
    private final Quantity R;
    private final Quantity K_T;
    private final Quantity K_W;

    private MotorConstants(double b, double R, double Kt, double Kw) {
        B = new Quantity(b, Units.Nm.divide(Units.RAD_PER_S));
        this.R = new Quantity(b, Units.Ohm);
        K_T = new Quantity(Kt, Units.Nm.divide(Units.A));
        K_W = new Quantity(Kw, Units.V.divide(Units.RAD_PER_S));
    }
}