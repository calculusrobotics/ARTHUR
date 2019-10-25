package frc.robot.utils.control.motor;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.control.ControlType;

import frc.robot.utils.control.encoder.QuadratureEncoder;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class BBTalonSRX extends BBMotorController {
    private final WPI_TalonSRX MOTOR;



    private int profileSlot0 = 0;
    private int profileSlot1 = 0;



    public BBTalonSRX(int deviceID) {
        MOTOR = new WPI_TalonSRX(deviceID);
    }



    @Override
    public void setPosition_ticks(int ticks, ControlType.Position controlMethod) {
        ControlMode mode;

        switch (controlMethod) {
            case PID: {
                mode = ControlMode.Position;
                break;
            }
            case MotionMagic: {
                mode = ControlMode.MotionMagic;
                break;
            }
            default: {
                return;
            }
        }

        MOTOR.set(mode, ticks);
    }

    @Override
    protected void addQuadratureEncoder(QuadratureEncoder sensor) {
        MOTOR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        MOTOR.setSelectedSensorPosition(0);
    }

    @Override
    public int getPosition_ticks() {
        return MOTOR.getSelectedSensorPosition();
    }


    
    @Override
    public double getVoltage() {
        // TODO: getBusVoltage() or MOTOR.getMotorOutputVoltage()?
        return MOTOR.getBusVoltage();
    }

    @Override
    public double getPercentVoltage() {
        // TODO: verify this = getVoltage() / RobotController.getBatteryVoltage()
        return MOTOR.getMotorOutputPercent();
    }
}