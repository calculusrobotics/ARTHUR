package frc.robot.utils.control.motor;



import frc.robot.utils.control.ControlType;
import frc.robot.utils.control.encoder.QuadratureEncoder;

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



    private CANEncoder encoder;



    public BBSparkMax(int deviceID, MotorType type) {
        MOTOR = new CANSparkMax(deviceID, type);

        PID_CONTROLLER = MOTOR.getPIDController();
    }



    @Override
    public void setPosition_ticks(int ticks, ControlType.Position controlMethod) {
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

        PID_CONTROLLER.setReference(revs, mode);
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
    public double getVelocity_ticks_per() {
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
}