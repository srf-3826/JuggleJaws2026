// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCmd;
import frc.robot.commands.DefaultShooterCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final PowerDistribution m_pdp = new PowerDistribution(0, ModuleType.kCTRE);
  private final CommandXboxController m_cmdXbox = new CommandXboxController(0);
  private final CANBus                m_canBus = new CANBus("rio");

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drivetrain = new DriveSubsystem(m_pdp);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem(m_canBus, m_pdp);     // , m_cmdXbox

  private SendableChooser<String> m_driveEnableChooser = new SendableChooser<String>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Assign default commands
    m_drivetrain.setDefaultCommand(
        new DefaultDriveCmd(() -> -m_cmdXbox.getLeftY(),
                            () -> -m_cmdXbox.getLeftX(), 
                            () -> getDriveEnabled(),
                            m_drivetrain));

    m_shooter.setDefaultCommand(
        new DefaultShooterCmd(() -> m_cmdXbox.getLeftTriggerAxis(), // launch motor, 0 - 1 range
                              () -> m_cmdXbox.getRightY(),          // feed motor, + / - dir
                              m_shooter));

    // Use sendableChooser on Dashboard to disable / enable differential driveBase
    // Allows general public to use game controller to launch balls without
    // risk of the robot moving.
    m_driveEnableChooser.setDefaultOption("Enabled", "Yes");
    m_driveEnableChooser.addOption("Disabled", "No");
    SmartDashboard.putData("Drive Disable", m_driveEnableChooser);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use configureButtonBindings to define your button->command mappings. 
   * CommandXboxController conveniently defines buttons as triggers; if needed use
   * .getHID to revert to native nutton / joystick valuess.
   */
  private void configureButtonBindings() {
  //
  // UI Control Summary:
  //
  // Drive:
  //   left joystick Y axis = drive forward/back
  //   left joystick X axis = normal steer left/right for larger Y axis
  //                        = rotate in place if Y axis value small
  //   Right Bumper = Interactive Slow mode. Sets maxOutput to .5 while held,
  //                  canceled on release.
  //   NOTE: drive can be disabled via a sendable chooser! Allows guest to control
  //         ball handling wihtout risk of hurting someone via robot movement.
  //         Also handy to ensure robot cannot be driven off of a parade trailer.
  // 
  // Shooter:
  //  DefaultShooterCmd - provides a manual override of the current feed and launch speeds.
  //     Left trigger axis = run launch motor at scaled axis rpm over full range
  //                         (axis 0 - 1 values are scaled to 0 - maxLaunchRpm).
  //     Right joystick Y axis = feed motor speed - range is -1.0 to +1.0 percent output
  //    NOTE: defaultShooterCmd is allowed in all states but JUGGLING. When the respective
  //    axis value returns to 0.0, the prior feed speed or launch RPMs will be retored.
  //
  //   Ball launch:
  //     Y button      = launch one ball (or timeout) at fixed feed speed and fixed launch speed
  //     ALT-Y button  = launch continuoously at fixed feed speed and fixed launch speed
  //     A button      = launch one ball at variable feed speed and variable launch speed
  //     ALT-A button  = launch continuously variable feed speed and variable launch speed
  //     B button      = continuous JUGGLE mode (catch own shot, re-cycle until cancelled)
  //     X button      = stop all shooter activity, return state to IDLE
  //
  //   Dpad Up    = increase variable Launch RPM by 200
  //   Dpad down  = decrease variable Launch RPM by 200
  //   Dpad Right = increase variable feed speed (percent out) by 0.2
  //   Dpad Left  = decrease variable feed speed (percent out) by 0.2
  //   Start button = unused
  //   Back button = unused
  //   Left Bumper = ALT
  

    // Connect the buttons to commands
    final Trigger ALT = m_cmdXbox.leftBumper();

    // Activate and disable slow mode
    m_cmdXbox.rightBumper()
      .onTrue(new InstantCommand(()-> m_drivetrain.setMaxDriveOutput(0.4), m_drivetrain));
    m_cmdXbox.rightBumper()
      .onFalse(new InstantCommand(()-> m_drivetrain.setMaxDriveOutput(1.0), m_drivetrain));

    // Launch single and continuous balls at fixed RPMs (from Constants.java)
    m_cmdXbox.y().and(ALT.negate()).onTrue(new InstantCommand(()->m_shooter.launchBallFixedRpm(false), m_shooter));
    ALT.and(m_cmdXbox.y()).onTrue(new InstantCommand(()->m_shooter.launchBallFixedRpm(true), m_shooter));
    
    // Launch single and continuous balls at variable RPMs (adjustable via DPad)
    m_cmdXbox.a().and(ALT.negate()).onTrue(new InstantCommand(()-> m_shooter.launchBallVariableRpm(false), m_shooter));
    ALT.and(m_cmdXbox.a()).onTrue(new InstantCommand(()-> m_shooter.launchBallVariableRpm(true), m_shooter));

    // Adjust vaariable RPMs for both Shooter and Feeder
    m_cmdXbox.povUp().onTrue(new InstantCommand(()-> m_shooter.increaseVariableLaunchRpm(), m_shooter));
    m_cmdXbox.povDown().onTrue(new InstantCommand(()-> m_shooter.decreaseVariableLaunchRpm(), m_shooter));
    m_cmdXbox.povRight().onTrue(new InstantCommand(()-> m_shooter.increaseVariableFeedSpeed(), m_shooter));
    m_cmdXbox.povRight().onTrue(new InstantCommand(()-> m_shooter.decreaseVariableFeedSpeed(), m_shooter));

    // Start juggling. Will continue until stopped (X) or until a Launch mode (above) is initiated.
    m_cmdXbox.b().onTrue(new InstantCommand(()-> m_shooter.startJuggling(), m_shooter));

    // Stop whatever is active (Juggling or Launching)
    m_cmdXbox.x().onTrue(new InstantCommand(()-> m_shooter.stopMotors(), m_shooter));
  }

  public boolean getDriveEnabled() {
    return (m_driveEnableChooser.getSelected() == "Yes"); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
