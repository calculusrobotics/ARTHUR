/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.utils.control.encoder.QuadratureEncoder;
import frc.robot.utils.control.motor.BBTalonSRX;
import frc.robot.utils.control.statespace.modeling.ildata.ILData;
import frc.robot.utils.math.units.Units;
import frc.robot.utils.roborio.RoboRIOFS;
import frc.robot.utils.math.units.Quantity;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();



    private BBTalonSRX[] talon = new BBTalonSRX[4];
    private ILData[] ilData = new ILData[4];



    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
        SmartDashboard.putBoolean("spin", false);





        for (int i = 0; i < 4; i++) {
            talon[i] = new BBTalonSRX(i + 1);
            talon[i].setRadius(new Quantity(2, Units.IN));
            talon[i].addEncoder(new QuadratureEncoder(QuadratureEncoder.EncoderType.AMT));

            talon[i].getTalonSRX().configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
            talon[i].getTalonSRX().setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 10);

            ilData[i] = new ILData(talon[i], 0.75);
        }


        RoboRIOFS.init();

        // PID constants = new PID(
        //     ControlType.PID,
        //     0.01,
        //     0,
        //     0
        // );

        // talon.addPID(constants);
        // // command two rotations using MotioMagic
        // talon.cmdPosition(2, ControlType.PID);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
        case kCustomAuto:
            // Put custom auto code here
            break;
        case kDefaultAuto:
        default:
        // Put default auto code here
            break;
        }
    }

    @Override
    public void teleopInit() {
        for (int i = 0; i < 4; i++) {
            ilData[i].run();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        boolean spin = SmartDashboard.getBoolean("spin", false);
        /*if (spin) {
            talon.cmdPercent(0.2);
        } else {
            talon.cmdPercent(0);
        }*/

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        
    }
}
