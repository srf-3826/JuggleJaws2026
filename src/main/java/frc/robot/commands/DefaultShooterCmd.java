// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.UI;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCmd extends Command {
  private final ShooterSubsystem m_shooter;
  private final DoubleSupplier m_launchSpeedSupplier;
  private final DoubleSupplier m_feedSpeedSupplier;
  
  private double  m_feedSpeed;
  private double  m_priorFeedSpeed;
  private boolean m_feedSpeedOverrideActive;

  private double  m_launchSpeed;
  private double  m_priorLaunchSpeed;
  private boolean m_launchSpeedOverrideActive;
  
  private int     m_state;

  /**
   * Create a new DefaultShooter command.
   *
   * @param launchSpeedSource launch motor manual controller source (a trigger axis, 0 to 1)
   * @param feedSpeedSource feed motor controller source            (a full axis, -1 to 1)
   */
  public DefaultShooterCmd(DoubleSupplier launchSpeedSource,
                           DoubleSupplier feedSpeedSource,
                           ShooterSubsystem shooterSubsystem) {
    m_shooter = shooterSubsystem;
    m_launchSpeedSupplier = launchSpeedSource;
    m_feedSpeedSupplier = feedSpeedSource;
    addRequirements(m_shooter);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Manual launch and feed control is allowed in any mode except JUGGLING.
    // If JUGGLING, or CHANGING_RPM_TO_JUGGLE, just return
     m_state = m_shooter.getCurrentState();
    if ((m_state == Shooter.JUGGLING) || (m_state == Shooter.CHANGING_RPM_TO_JUGGLE)) {
      return;
    }

    // Otherwise, get axis Values, apply deadband
    m_launchSpeed = m_launchSpeedSupplier.getAsDouble();
    m_feedSpeed = -m_feedSpeedSupplier.getAsDouble();
        // First suppress launchSpeed and/or feedSpeed if less than respective deadbands
    if (Math.abs(m_feedSpeed) <= UI.JOYSTICK_DEADBAND) {
      m_feedSpeed = 0.0;
    }
    if (m_launchSpeed <= UI.TRIGGER_DEADBAND) {
      m_launchSpeed = 0.0;
    }

    // Because JuggleJaws can be set to fire at various shooter and feed speeds,
    // wether single shot or continuously, then having manual joystick drive available
    // means we need to treat it aas an override, and return to the previously set
    // values when the joysticks  are released.
    // Thus for both Feed speed and Launch speed, but processed independently,
    // when the respective axis value is non zero, and the override flag is 
    // not set, then set it and store the current motor speed. If the override
    // flag is set, then continue driving the respective motor at the manual Joystick value.
    // When the respective axis value returns to 0.0 (i.e. below the deadband), then
    // check if the override flag is set. If so, clear it and restore the 
    // respective motor speed to the prior stored value and exit.
    if (Math.abs(m_feedSpeed) > UI.JOYSTICK_DEADBAND) {
      if (! m_feedSpeedOverrideActive) {
        m_feedSpeedOverrideActive = true;
        m_priorFeedSpeed = m_shooter.getTargetFeedSpeed();
      }
      m_shooter.driveFeedMotor(m_feedSpeed);
    } else {                                      // Feed Speed has dropped below deadband. 
      if (m_feedSpeedOverrideActive) {
        m_feedSpeedOverrideActive = false;
        m_shooter.driveFeedMotor(m_priorFeedSpeed);
      }
    }

    if (m_launchSpeed > 0.0) {
      if (! m_launchSpeedOverrideActive) {
        m_launchSpeedOverrideActive = true;
        m_priorLaunchSpeed = m_shooter.getTargetLaunchSpeed();
      }
      m_launchSpeed *= Shooter.MAX_LAUNCH_RPM;
      m_shooter.driveLaunchMotor(m_launchSpeed);
    } else {
      if (m_launchSpeedOverrideActive) {
        m_launchSpeedOverrideActive = false;
        m_shooter.driveLaunchMotor(m_priorLaunchSpeed);
      }
    }
  }
    
  // This default command can be interrupted, but never terminates on its own
  @Override
  public boolean isFinished() {
    return false;
  }
}
