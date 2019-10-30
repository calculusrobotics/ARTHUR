package frc.robot.utils.control.motor;



import frc.robot.utils.control.controltype.ControlType;
import frc.robot.utils.control.encoder.QuadratureEncoder;
import frc.robot.utils.control.pidf.PID;
import frc.robot.utils.control.pidf.PIDF;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;



/**
 * Wrapper class for CANSparkMax motors
 */
public class BBSparkMax extends BBMotorController {
    private final CANSparkMax MOTOR;
    private final CANPIDController PID_CONTROLLER;

    private int pidSlot;



    private CANEncoder encoder;



    public BBSparkMax(int deviceID, MotorType type) {
        MOTOR = new CANSparkMax(deviceID, type);

        PID_CONTROLLER = MOTOR.getPIDController();
    }



    @Override
    public void trySetPID(int pidSlot) {
        PID pid = pidConstants.get(pidSlot);

        PID_CONTROLLER.setP(pid.getKP(), pidSlot);
        PID_CONTROLLER.setI(pid.getKI(), pidSlot);
        PID_CONTROLLER.setD(pid.getKD(), pidSlot);

        if (pid instanceof PIDF) {
            PIDF pidf = (PIDF) pid;

            PID_CONTROLLER.setFF(pidf.getKF(), pidSlot);
        }
    }



    @Override
    public void setPIDSlot(int pidSlot) {
        this.pidSlot = pidSlot;
    }



    @Override
    public void cmdPosition_ticks(int ticks, ControlType controlMethod) {
        // RevRobotics also has a ControlType class :/
        com.revrobotics.ControlType mode;

        // get encoder revolutions because SparkMax uses revolutions as default unit
        double revs = ticksToRevs(ticks);

        switch (controlMethod) {
            case PID: {
                mode = com.revrobotics.ControlType.kPosition;
                break;
            }
            case MotionMagic: {
                mode = com.revrobotics.ControlType.kSmartMotion;
                break;
            }
            default: {
                return;
            }
        }

        PID_CONTROLLER.setReference(revs, mode, pidSlot);
    }

    @Override
    protected void addQuadratureEncoder(QuadratureEncoder sensor) {
        encoder = MOTOR.getEncoder(EncoderType.kQuadrature, sensor.getCPR());
        encoder.setPosition(0);
    }

    @Override
    public int getPosition_ticks() {
        // SparkMax returns position in revs
        return (int) (encoder.getPosition() * sensor.getTicksPerRev());
    }



    @Override
    protected double getTimePeriod() {
        return 60; // RPM -> minutes -> 60 seconds
    }



    @Override
    public double getVelocity_nu() {
        return revsToTicks(encoder.getVelocity()); // I'm somebody
    }



    @Override
    public double getVoltage() {
        return MOTOR.getBusVoltage();
    }

    @Override
    public double getPercentVoltage() {
        return getVoltage() / RobotController.getBatteryVoltage();
    }



    @Override
    public void follow(BBMotorController motorController) {
        if (motorController instanceof BBSparkMax) {
            CANSparkMax leader = ((BBSparkMax) motorController).getSparkMax();

            MOTOR.follow(leader);
        }
    }



    public CANSparkMax getSparkMax() { return MOTOR; }
}