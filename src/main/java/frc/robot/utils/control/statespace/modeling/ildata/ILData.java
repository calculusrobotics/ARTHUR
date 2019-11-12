package frc.robot.utils.control.statespace.modeling.ildata;

import frc.robot.utils.control.motor.BBMotorController;
import frc.robot.utils.control.statespace.Motors;
import frc.robot.utils.math.units.Units;
import frc.robot.utils.roborio.RoboRIOFS;
import edu.wpi.first.wpilibj.Notifier;

import java.io.File;
import java.io.IOException;
import java.io.FileWriter;
import java.io.PrintWriter;



/**
 * Measure moment of inertia (I) and inductance (L) through a motor by running it
 * at a set voltage. Warning: this requires changing all motor units to SI and
 * having no gear ratios or load.
 */
public class ILData {
    private static final double PERIOD_SEC = 0.005; // periodt

    private static final int MEASUREMENTS_PER_TRIAL = 30;
    private static final int TRIALS = 10;



    public static class DataPoint {
        private final double THETA;
        private final double OMEGA;
        private final double CURRENT;
        private final double CURRENT_DERIV;
        private final double VOLTAGE;

        private DataPoint(double theta, double omega, double current, double currentDeriv, double voltage) {
            THETA = theta;
            OMEGA = omega;
            CURRENT = current;
            CURRENT_DERIV = currentDeriv;
            VOLTAGE = voltage;
        }

        public String toString() {
            return THETA + " " + OMEGA + " " + CURRENT + " " + CURRENT_DERIV + " " + VOLTAGE;
        }
    }



    private static enum State {
        Measurement, // measurement state: constant voltage is applied. if voltage deviates from a
                     // constant value, move back to REST state and try again to get more accurate data
        Rest; // resting state: no voltage applied, slow down motors to 0 current and angular velocity
    }



    

    private final BBMotorController MOTOR;
    private final double PERCENT;

    private int trialNum = 0;

    private Notifier notifier;
    private State state = State.Rest;



    // measurement stuff
    private double lastVoltage = -1;
    private double lastCurrent = 0;
    private DataPoint[] dataPoints = new DataPoint[MEASUREMENTS_PER_TRIAL];
    private int measurementNum = 0;
    

    
    public ILData(BBMotorController motor, double percent) {
        MOTOR = motor;
        PERCENT = percent;



        MOTOR.setMeasurementToAngle();
        MOTOR.setThetaUnit(Units.RAD);
        MOTOR.setTimeUnit(Units.S);
        MOTOR.setSecondTimeUnit(Units.S);
        MOTOR.setInverted(false);
        MOTOR.setSensorPhase(false);
        MOTOR.setOpenLoopRampRate(0);
        MOTOR.zero();
    }



    public void run() {
        notifier = new Notifier(new Loop());
        notifier.startPeriodic(PERIOD_SEC); // 5 ms loop
    }



    private void writeMeasurements() {
        String pathName = RoboRIOFS.MOTOR_DATA_NAME + MOTOR.getDeviceID() + "/trial" + trialNum + ".txt";
        try {
            FileWriter fileWriter = new FileWriter(pathName);
            PrintWriter writer = new PrintWriter(fileWriter);

            for (int i = 0; i < MEASUREMENTS_PER_TRIAL; i++) {
                writer.println(dataPoints[i].toString());
            }

            writer.close();
        } catch (IOException e) {}
    }



    private class Loop implements Runnable {
        @Override
        public void run() {
            if (state == State.Measurement) {
                double voltage = MOTOR.getVoltage();

                if (lastVoltage == -1) {
                    lastVoltage = voltage;
                }

                if (voltage != lastVoltage) {
                    // rip
                }

                double theta = MOTOR.getPosition();
                double omega = MOTOR.getVelocity();
                double current = MOTOR.getCurrent();
                double currentDeriv = (current - lastCurrent) / PERIOD_SEC;

                DataPoint dp = new DataPoint(theta, omega, current, currentDeriv, voltage);
                dataPoints[measurementNum] = dp;

                measurementNum++;
                if (measurementNum == MEASUREMENTS_PER_TRIAL) {
                    writeMeasurements();

                    trialNum++;

                    MOTOR.cmdPercent(0); // rest the motor
                    state = State.Rest;

                    lastVoltage = -1;
                    lastCurrent = 0;
                    measurementNum = 0;
                }
            } else if (state == State.Rest) {
                double omega = MOTOR.getVelocity();
                double current = MOTOR.getCurrent();
                double voltage = MOTOR.getVoltage();

                if (omega == 0 && current == 0 && voltage == 0) {
                    if (trialNum == TRIALS) {
                        notifier.stop();
                    } else {
                        MOTOR.zero();
                        MOTOR.cmdPercent(PERCENT);
                    }
                }
            }
        }
    }
}