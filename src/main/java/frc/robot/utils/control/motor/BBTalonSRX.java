package frc.robot.utils.control.motor;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.math.MathUtils;

import frc.robot.utils.control.controltype.ControlType;
import frc.robot.utils.control.pidf.PID;
import frc.robot.utils.control.pidf.PIDF;
import frc.robot.utils.control.encoder.QuadratureEncoder;
import frc.robot.utils.control.motionprofile.motionmagic.MotionMagic;

import frc.robot.utils.math.units.Unit;
import frc.robot.utils.math.units.Units;
import frc.robot.utils.math.units.BaseUnit;

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
    protected void trySetPID(int pidID) {
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
    public void setPIDSlot(int pidSlot) {
        MOTOR.selectProfileSlot(pidSlot, 0);
    }



    @Override
    public void cmdPosition_ticks(double ticks, ControlType controlMethod) {
        ControlMode mode;

        int ticksInt = MathUtils.round(ticks);

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

        MOTOR.set(mode, ticksInt);
    }

    @Override
    protected void configMotionMagic_nu(double acc, double vel) {
        int accInt = (int) (acc + 0.5);
        int velInt = (int) (vel + 0.5);

        MOTOR.configMotionAcceleration(accInt);
        MOTOR.configMotionCruiseVelocity(velInt);
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
    protected BaseUnit getTimeUnit() {
        return Units.MS100; // ticksPer100ms -> 100ms
    }

    @Override
    protected BaseUnit getSecondTimeUnit() {
        return Units.S; // ticksPer100msPerSec -> 1s (ik its dumb that's CTRE tho)
    }

    @Override
    protected BaseUnit getNativeUnit() {
        if (sensor == null) {
            return null;
        }

        return new BaseUnit(Units.REV, sensor.getTicksPerRev());
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