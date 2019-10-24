package frc.robot.utils.control.encoder;

public class QuadratureEncoder extends SensorType {
    /*
     * For quadrature encoders, the ticks per revolution depends on the
     * type of encoder. TPR = 4 * CPR (quad comes from the 4)
     * where CPR is the internal counts per revolution to the
     * encoder.
     */

    /** Type of encoder */
    public enum EncoderType {
        AMT201 (2048);



        private final int CPR;
        private EncoderType(int cpr) {
            CPR = cpr;
        }

        private int getCPR() {
            return CPR;
        }
    }



    private final int CPR;

    public QuadratureEncoder(EncoderType encoderType) {
        super(4 * encoderType.getCPR());

        CPR = encoderType.getCPR();
    }



    // TODO: does SparkMax handle TPR = 4 * CPR the same way as TalonSRX?
    public int getCPR() {
        return CPR;
    }
}