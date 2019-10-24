package frc.robot.utils.control.motor;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.control.encoder.QuadratureEncoder;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class BBTalonSRX extends BBMotorController {
    private final WPI_TalonSRX MOTOR;



    private int profileSlot0 = 0;
    private int profileSlot1 = 0;



    public BBTalonSRX(int deviceID) {
        MOTOR = new WPI_TalonSRX(deviceID);
    }



    @Override
    protected void addQuadraticEncoder(QuadratureEncoder sensor) {
        MOTOR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        MOTOR.setSelectedSensorPosition(0);
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