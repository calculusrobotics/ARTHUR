package frc.robot.utils.control.motor;



import frc.robot.utils.control.encoder.QuadratureEncoder;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;



/**
 * Wrapper class for CANSparkMax motors
 */
public class BBSparkMax extends BBMotorController {
    private final CANSparkMax MOTOR;



    private CANEncoder encoder;



    public BBSparkMax(int deviceID, MotorType type) {
        MOTOR = new CANSparkMax(deviceID, type);
    }



    @Override
    protected void addQuadraticEncoder(QuadratureEncoder sensor) {
        encoder = MOTOR.getEncoder(EncoderType.kQuadrature, sensor.getCPR());
        encoder.setPosition(0);
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