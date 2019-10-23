package frc.robot.utils.control.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Wrapper class for CANSparkMax motors
 */
public class BBSparkMax extends BBMotorController {
    private final CANSparkMax MOTOR;



    public BBSparkMax(int deviceID, MotorType type) {
        MOTOR = new CANSparkMax(deviceID, type);
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