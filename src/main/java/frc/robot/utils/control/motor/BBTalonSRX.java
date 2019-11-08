package frc.robot.utils.control.motor;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.control.controltype.ControlType;
import frc.robot.utils.control.pidf.PID;
import frc.robot.utils.control.pidf.PIDF;
import frc.robot.utils.control.encoder.QuadratureEncoder;

import frc.robot.utils.math.units.Units;
import frc.robot.utils.math.units.BaseUnit;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;


public class BBTalonSRX extends BBMotorController {
    private final WPI_TalonSRX MOTOR;



    public BBTalonSRX(int deviceID) {
        MOTOR = new WPI_TalonSRX(deviceID);
    }



    protected void loadPID(int pidID, int slot) {
        PID pid = pidConstants.get(pidID);

        MOTOR.config_kP(pidID, pid.getKP());
        MOTOR.config_kI(pidID, pid.getKI());
        MOTOR.config_kD(pidID, pid.getKD());

        if (pid instanceof PIDF) {
            PIDF pidf = (PIDF) pid;

            MOTOR.config_kF(pidID, pidf.getKF());
        }
    }

    @Override
    protected int getMaxPIDSlots() { return 2; }



    @Override
    public void selectPIDSlot(int pidSlot) {
        MOTOR.selectProfileSlot(pidSlot, 0);
    }



    @Override
    public void cmdPosition_native(double val_nu, ControlType controlMethod) {
        ControlMode mode;

        int ticks = (int) Math.round(val_nu);

        switch (controlMethod) {
            case Position: {
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
    public void cmdPercent_native(double perc) {
        MOTOR.set(ControlMode.PercentOutput, perc);
    }

    @Override
    protected void configMotionMagic_nu(double acc, double vel) {
        int accInt = (int) Math.round(acc);
        int velInt = (int) Math.round(vel);

        MOTOR.configMotionAcceleration(accInt);
        MOTOR.configMotionCruiseVelocity(velInt);
    }

    @Override
    protected void addQuadratureEncoder(QuadratureEncoder sensor) {
        MOTOR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        MOTOR.setSelectedSensorPosition(0);
        MOTOR.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100);
    }

    @Override
    public double getPosition_nu() {
        return MOTOR.getSelectedSensorPosition();
    }



    @Override
    protected BaseUnit getTimeUnit_nu() {
        return Units.MS100; // ticksPer100ms -> 100ms
    }

    @Override
    protected BaseUnit getSecondTimeUnit_nu() {
        return Units.S; // ticksPer100msPerSec -> 1s (ik its dumb that's CTRE tho)
    }

    @Override
    protected BaseUnit getThetaUnit_nu() {
        if (sensor == null) {
            return null;
        }

        return new BaseUnit(Units.REV, sensor.getTicksPerRev(), "tick");
    }



    @Override
    public double getVelocity_nu() {
        return MOTOR.getSelectedSensorVelocity(); // thank you CTRE for not using revs per min unlike SOMEBODY
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



    @Override
    public void follow(BBMotorController motorController) {
        if (motorController instanceof BBTalonSRX) {
            WPI_TalonSRX leader = ((BBTalonSRX) motorController).getTalonSRX();

            MOTOR.follow(leader);
        }
    }



    public WPI_TalonSRX getTalonSRX() { return MOTOR; }
}