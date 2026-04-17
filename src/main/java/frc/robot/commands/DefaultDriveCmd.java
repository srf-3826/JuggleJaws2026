// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Have the robot drive arcade style.
 * But add a feature whereby "rotate in place" is possible with low
 * enough forward / reverse joystick values.
 */
public class DefaultDriveCmd extends Command {
  private final DriveSubsystem m_drivetrain;
  private final DoubleSupplier m_speedSource;
  private final DoubleSupplier m_rotateSource;
  private final BooleanSupplier m_driveEnabledSource;
  private       double         m_speedInput;
  private       double         m_rotateInput;
  private       boolean        m_driveSafetyIsEnabled = true;

  /**
   * Creates a new ArcadeDrive command.
   *
   * @param speed The control input for forward / back movement
   * @param rotate The control input for the direction - left or right
   * @param drivetrain The drivetrain subsystem to drive
   */
  public DefaultDriveCmd(DoubleSupplier speed, 
                         DoubleSupplier rotate,
                         BooleanSupplier driveEnabled, 
                         DriveSubsystem drivetrain) {
    m_speedSource = speed;
    m_rotateSource = rotate;
    m_driveEnabledSource = driveEnabled;
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (m_driveEnabledSource.getAsBoolean()) {
      // Sendable chooser DriveEnabled has been switched to "Yes"
      // If motor timeout safety has previously been disabled, enable it now
      if (! m_driveSafetyIsEnabled) {
        m_drivetrain.enableDriveSafety();
        m_driveSafetyIsEnabled = true;
      }
      // and read actual joystick drive commands to send to the Drive subsystem
      m_speedInput = m_speedSource.getAsDouble();
      m_rotateInput = m_rotateSource.getAsDouble();
    } else {
      // Sendable Chooser Drive Enabled has been set to "No"
      // If motor timeout safety is currently enabled, turn it off
      if (m_driveSafetyIsEnabled) {
        m_drivetrain.disableDriveSafety();
        m_driveSafetyIsEnabled = false;
      }
      // simulate null joystick input to send to the Drive Subsystem
      m_speedInput = 0.0;
      m_rotateInput = 0.0;
    }
    m_drivetrain.drive(m_speedInput, m_rotateInput);
  }

  // Called once after isFinished returns true, or is interrupted
  @Override
  public void end(boolean interrupted) {
    // m_drivetrain.drive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }
}
