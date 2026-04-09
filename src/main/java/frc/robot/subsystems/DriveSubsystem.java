// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PowerDistribution;

// Comment out following line if gyro is not present
// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {
  /**
   * The Drivetrain subsystem incorporates the sensors and actuators attached to the robots chassis.
   * These include two drive motors, no encoders, and an optional gyro.
   */
  private WPI_TalonSRX      m_leftMotor;
  private WPI_TalonSRX      m_rightMotor;
  private DifferentialDrive m_drive;
  private double            m_maxDriveOutputFactor = DiffDrive.DRIVE_OUTPUT_LIMIT;
  
  private final PowerDistribution m_pdp;

 // Comment out the following line if no gyro is being used
 // private final AHRS m_gyro;

  /** Create a new drivetrain subsystem. */
  public DriveSubsystem(PowerDistribution pdp) {
    super("DriveSubsystem");

    m_pdp = pdp;

    m_leftMotor = new WPI_TalonSRX(DiffDrive.LEFT_MOTOR_CTRL_CAN_ID);
    m_rightMotor = new WPI_TalonSRX(DiffDrive.RIGHT_MOTOR_CTRL_CAN_ID);
    m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    // configure drive motors
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Here we invert the right side.
    m_leftMotor.configFactoryDefault();
    m_leftMotor.configSupplyCurrentLimit(DiffDrive.driveSupplyCurrentLimit);
    m_leftMotor.setInverted(false);
    m_leftMotor.setNeutralMode(NeutralMode.Brake);    // Was Coast

    m_rightMotor.configFactoryDefault();
    m_rightMotor.configSupplyCurrentLimit(DiffDrive.driveSupplyCurrentLimit);
    m_rightMotor.setInverted(true);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);   // was Coast

    setMaxDriveOutput(DiffDrive.DRIVE_OUTPUT_LIMIT);    // set default max output - redundant...
  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    // SmartDashboard.putNumber("Gyro", getHeading());
    SmartDashboard.putNumber("Max Drive Output", m_maxDriveOutputFactor);
    SmartDashboard.putNumber("LD Motor Amps = ", m_leftMotor.getSupplyCurrent());
    SmartDashboard.putNumber("LD PDP Amps = ", m_pdp.getCurrent(DiffDrive.LEFT_DRIVE_MOTOR_PDP_SLOT));
    SmartDashboard.putNumber("RD Motor Amps = ", m_rightMotor.getSupplyCurrent());
    SmartDashboard.putNumber("RD PDP Amps = ", m_pdp.getCurrent(DiffDrive.RIGHT_DRIVE_MOTOR_PDP_SLOT));
  }

  /**
   * drive is a method to turn joystick input values to standard WpiLib
   * arcade driving. Uncomment either arcadeDrive or curvatureDrive below to 
   * enable the version is desired, and to compare them against each other.
   *
   * WPILib methods turn these parms into PercentOutput, and write them
   * directly to the TalonSRX motor controllers. 
   * @param speed Speed in range [-1,1]. 
   * @param direction Curvature (or SteeringAngle for turn in place if 
   *                             speed is low enough - range is [-1,1]
   */
  public void drive(double speed, double direction) {
    //SmartDashboard.putNumber("Drive; Speed = ", speed);
    //SmartDashboard.putNumber("Drive; Direction = ", direction);
    m_drive.arcadeDrive(speed, direction, true);
  }

  // setMaxDriveOutput sets a speed reduction factos, typically called 
  // on right bumper button presses/releases.
  public void setMaxDriveOutput(double controllerFactor) {
    m_maxDriveOutputFactor = controllerFactor;
    m_drive.setMaxOutput(controllerFactor);
  }

  // If the driveEnable sendable chooser in SmartDashboard is switched (whether to disaabled, 
  // or to enabled) the following two support methods (called from defaultDriveCmd) ensure 
  // that drive motors safety is set to match the current choice.
  public void enableDriveSafety() {
    m_drive.setSafetyEnabled(true);
  }

  public void disableDriveSafety() {
    m_drive.setSafetyEnabled(false);
  }
  /**
   * Get the robot's heading.
   *
   * @return The robot's heading in degrees.
   */
  public double getHeading() {
    // return m_gyro.getAngle();
    return 0.0;
  }

  /** Reset the robots sensors to the zero states. */
  public void reset() {
    // m_gyro.reset();
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
  }
}
