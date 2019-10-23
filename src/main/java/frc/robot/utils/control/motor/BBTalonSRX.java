package frc.robot.utils.control.motor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class BBTalonSRX extends BBMotorController {
    private final WPI_TalonSRX MOTOR;



    public BBTalonSRX(int deviceID) {
        MOTOR = new WPI_TalonSRX(deviceID);
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