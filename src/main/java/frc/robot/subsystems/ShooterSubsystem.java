// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants.*;
import frc.robot.Utils.Utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The ball shooter subsystem uses PID to go to a given RPM, and when the setpoint
 * is reached, it starts the feed motor to drive the ball into the shooter wheels.
 */
public class ShooterSubsystem extends SubsystemBase {
  private TalonFX       m_launchMotor;
  private double        m_variableLaunchRpm = Shooter.WORKING_LAUNCH_RPM;     // default. Can be adjusted via Dpad up or down
  private double        m_measuredLaunchRpm;
  private double        m_targetLaunchRpm = Shooter.WORKING_LAUNCH_RPM;       // Most recent requested launch speed
  private double        m_juggleRpm = Shooter.JUGGLE_LAUNCH_RPM;
  private double        m_statorLaunchCurrentDirect;
  private double        m_statorLaunchCurrentIndirect;
  private double        m_supplyLaunchCurrentDirect;
  private double        m_supplyLaunchCurrentIndirect;
  private double        m_supplyLaunchCurrentDirectBaseline = 0.0;
  private double        m_pdpLaunchSupplyCurrent;
  private StatusSignal<Double>  m_launchStatorStatusSignal;
  private StatusSignal<Double>  m_launchSupplyStatusSignal;

  private TalonSRX      m_feedMotor;
  private double        m_targetFeedSpeed = Shooter.WORKING_FEED_SPEED;
  private double        m_variableFeedSpeed = Shooter.WORKING_FEED_SPEED;
  private double        m_avgMeasuredRpm = 0.0; 
 
  private boolean       m_continuousFlag = false;

  // Phoenix6 control mode for Launch motor
  private final VelocityVoltage m_velocityVoltage = 
                  new VelocityVoltage(0,                    // velocity - to be set with each invocation
                                      0,                // Acceleration - to be set with each invocation, or left as default
                                      true,                // enable FOC
                                      2.0,               // feedforward - volts needed to drive to desired setpoint steady state
                                      Shooter.LAUNCH_PID_SLOT,
                                      false,  // Overide brake during newtral
                                      false,       // Limit forward motion
                                      false        // Limit reverse motion
                                     );
  // VelocityVoltage control with a velocity of 0.0 does not stop the motor.
  // Nor does m_launchMotor.set(0.0).   Use this control mode instead.
  private final NeutralOut m_brake = new NeutralOut();

  // Time variables for various timeouts. Timeouts always need a start time. Also used are the current time ("timeNow")
  // and calculated elapsed time. Using member variables for these latter values can avoid garbage collection issues .
  private long            m_timeMs;
  // start time and elapsed time for the timeout monitoring changing the speed of launcher wheels. If tsrget rpm is not
  // achieved within expected time, the motor is stopped.
  private long            m_launcherStartTimeMs;  
  private long            m_elapsedSpinUpTimeMs;
  // Start and elapsed times for ball launch timeout. With a ball anywhere in the queue, starting the feed 
  // motor should get the ball to the shooter launch wheels in less than the timeout duration.
  // Motor current is used to sense an actual ball launch, which in turn will trigger a feed motor stop to avoid
  // firing multiple balls per launch trigger.
  private long            m_feederStartTimeMs;   
  private long            m_elapsedLaunchTimeMs;
  
  // Declare a state variable, and start in the IDLE state.
  private int             m_shooterState = Shooter.IDLE;

  private final PowerDistribution m_pdp;
  // private CommandXboxController   m_cmdXbox;

  /** Create a new ball shooter subsystem. */
  public ShooterSubsystem(PowerDistribution pdp) {            // , CommandXboxController cmdXbox
    m_pdp = pdp;
    // m_cmdXbox = cmdXbox;

    m_feedMotor = new TalonSRX(Shooter.FEED_MOTOR_CTRL_CAN_ID);
    configFeedMotor();
  
    m_launchMotor = new TalonFX(Shooter.LAUNCH_MOTOR_TALON_FX_CAN_ID, "rio");
    configLaunchMotor();

    m_launchStatorStatusSignal = m_launchMotor.getStatorCurrent();
    m_launchSupplyStatusSignal = m_launchMotor.getSupplyCurrent();
  }

  private void configFeedMotor() {
    m_feedMotor.configFactoryDefault();
    // m_feedMotor.configAllSettings(ctreConfigs.feedSRXConfig);
    m_feedMotor.configSupplyCurrentLimit(Shooter.feedSupplyLimit);
    m_feedMotor.setInverted(true);
    m_feedMotor.setNeutralMode(NeutralMode.Coast);
    // TO DO: set current limits, soft limit switches
  } 

  private void configLaunchMotor() {
    var launchOutputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                                                     .withInverted(InvertedValue.CounterClockwise_Positive)
                                                     .withPeakForwardDutyCycle(1.0)
                                                     .withPeakReverseDutyCycle(-1.0);
    var voltageConfig = new VoltageConfigs().withPeakForwardVoltage(8.5)      // ~ 4250 RPM
                                            .withPeakReverseVoltage(-8.5);
    var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                              .withFeedbackRemoteSensorID(0)
                                              .withSensorToMechanismRatio(1.0)    // No gear box - direct drive
                                              .withRotorToSensorRatio(1.0);
    var pid0Config = new Slot0Configs().withKP(0.2)
                                       .withKI(0.0)
                                       .withKD(0.0001)
                                       .withKS(0.0)
                                       .withKV(0.12)
                                       .withKA(0.0)
                                       .withKG(0.0);
    var openLoopRampConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0.25)
                                                       .withVoltageOpenLoopRampPeriod(0.25)
                                                       .withTorqueOpenLoopRampPeriod(0);
    var closedLoopRampConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0.24)
                                                           .withVoltageClosedLoopRampPeriod(0.25)
                                                           .withTorqueClosedLoopRampPeriod(0);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(30.0)
                                                       .withSupplyCurrentThreshold(60.0)
                                                       .withSupplyTimeThreshold(0.1)
                                                       .withSupplyCurrentLimitEnable(true)
                                                       .withStatorCurrentLimit(40.0)
                                                       .withStatorCurrentLimitEnable(true);
    var launchConfig = new TalonFXConfiguration().withMotorOutput(launchOutputConfig)
                                                 .withVoltage(voltageConfig)
                                                 .withFeedback(feedbackConfig)
                                                 .withSlot0(pid0Config)
                                                 .withOpenLoopRamps(openLoopRampConfig)
                                                 .withClosedLoopRamps(closedLoopRampConfig)
                                                 .withCurrentLimits(currentLimitConfig);
    StatusCode status = m_launchMotor.getConfigurator().apply(launchConfig);

    if (! status.isOK() ) {
        System.out.println("Failed to apply launch Motor configs. Error code: "+status.toString());
    }
  }

  // changeState is a Utility routine that changes state, but also ensures a record is kept
  // showing all changes, including the immediately previous state.
  public void changeState(int newState) {
    SmartDashboard.putNumber("Shooter State = ", newState);
    System.out.println("Change State from "+m_shooterState+" to "+newState);
    m_shooterState = newState;
  }

  // decreaseVariableLaunchRpm reduces m_launchRpm by 200 RPM, if the result will
  // be equal to or greater than MIN_LAUNCH_RPM
  public void decreaseVariableLaunchRpm() {
    if (m_shooterState == Shooter.JUGGLING) {
      // when JUGGLING, adjust the currently active Launch RPM by just 100 and apply the 
      // change immediately, but limit the max RPM to JUGGLE_LAUNCH_RPM, and the min RPM to 
      // (JUGGLE_LAUNCH_RPM - 300). This allow fine tuning of the Juggle to reduce chances
      // of the ball overshooting the basket during parades or other robot movement while
      // juggling.
      if (m_juggleRpm == (Shooter.JUGGLE_LAUNCH_RPM)) {
        m_juggleRpm -= 100.0;
        driveLaunchMotor(m_juggleRpm);
      }
    } else {
      // Regular launch 200 RPM adjustment - do in background, and do not apply to motor
      // until a launch button (a, y, ALT-a, or ALT-y) is pressed.
      if (m_variableLaunchRpm >= (Shooter.MIN_LAUNCH_RPM + 200.0)) {
        m_variableLaunchRpm -= 200.0;
      }
    }
  }

 // increaseDefaultLaunchRpm increases m_defaultLaunchRpm by 200 RPM, if the result will
 // be equal to or less than MAX_LAUNCH_RPM
  public void increaseVariableLaunchRpm() {
    if (m_shooterState == Shooter.JUGGLING) {
      if (m_juggleRpm == (Shooter.JUGGLE_LAUNCH_RPM - 100)) {
        m_juggleRpm += 100.0;
        driveLaunchMotor(m_juggleRpm);
      }
    } else {      
    }
    if (m_variableLaunchRpm <= Shooter.MAX_LAUNCH_RPM - 200.0) {
      m_variableLaunchRpm += 200.0;
    }
  }

  public double getTargetLaunchSpeed() {
    return m_targetLaunchRpm;
  }

  public void decreaseVariableFeedSpeed() {
    if (m_variableFeedSpeed >= (Shooter.MIN_FEED_SPEED + 0.2)) {
      m_variableFeedSpeed -= 0.2;
    }
  }

  public void increaseVariableFeedSpeed() {
    if (m_variableFeedSpeed <= (Shooter.MAX_FEED_SPEED - 0.2)) {
      m_variableFeedSpeed += 0.2;
    }
  }
  
  public double getTargetFeedSpeed() {
    return m_targetFeedSpeed;
  }

  public int getCurrentState() {
    return m_shooterState;
  }

  // getLaunchMotorRpm returns the current RPM of the launch motor.
  // It reads the internal encoder (which returns units of countsd per 100 ms)
  // and converts that to RPM to be returned
  public double getLaunchMotorRpm() {
    return Utils.convertRpsToRpm(m_launchMotor.getVelocity().getValueAsDouble());
  }

  public void launchBallVariableRpm(boolean continuousFlag) {                    // a or ALT-a (ALT = continuous)
    launchBall(m_variableLaunchRpm, m_variableFeedSpeed, continuousFlag);
  }

  public void launchBallFixedRpm(boolean continuousFlag) {                       // y or ALT-y (ALT = continuous)
    launchBall(Shooter.WORKING_LAUNCH_RPM, Shooter.WORKING_FEED_SPEED, continuousFlag);
  }

  // launchBall changes to a ball launch mode, either with a feed timeout, or continuous.
  // It is allowed in any state, even JUGGLING or CHANGING_RPM, as well as LAUNCHING (because
  // launch speed or feed speed may change).
  // This is a "start the process" routine, setting state and sending motor controller instructions
  // to change speed(s). The periodic() method must take over after the target RPM is reached 
  // in order to start the feed motor which will move a ball into contact with the shooter wheels.
  // periodic() will also stop the feed motor when the ball is considered to have been launched, 
  // based on the shooter moter experiencing a supply or stator current spike, or via a timeot,
  // unless the continuousFlag is set, in which case the mode will run until canceled.
  //
  public void launchBall(double launchRpm, double feedSpeed, boolean continuousFlag) {
    m_continuousFlag = continuousFlag;
    if ((m_shooterState == Shooter.LAUNCHING) || (m_shooterState == Shooter.READY_TO_LAUNCH)) {
      if (feedSpeed != m_targetFeedSpeed) {
        m_targetFeedSpeed = feedSpeed;    // might remain applied now, or suppressed until RPM is reached
        startFeedMotor(feedSpeed);
      }
      if (launchRpm == m_targetLaunchRpm) {
        startFeedMotor(feedSpeed);
        changeState(Shooter.LAUNCHING);
        return;
      }
    }
    stopFeedMotor();
    changeState(Shooter.CHANGING_RPM_TO_LAUNCH);
    startLaunchMotor(launchRpm);
  }

  // startJuggling is processed in any state except (already) JUGGLING or CHANGING_RPM_TO_JUGGLE,
  // Called as an InstantCommand from RobotContainer when the B button is pressed.
  // To process, stop the feedMotor, then start the launch motor at m_juggleRpm (initialized
  // to JUGGLE_LAUNCH_RPM), and set the state to CHANGING_RPM_TO_JUGGLE. When the RPM is reached then 
  // the periodic() method starts the feed motor and sets the State to JUGGLING
  // where it will stay until manually cancelled (via the X button), or superceded
  // with one of the four available launch modes (Y, A, ALT-Y, and ALT-A). 
  public void startJuggling() {
    if (m_shooterState == Shooter.JUGGLING) {
      return;
    }
    stopFeedMotor();
    changeState(Shooter.CHANGING_RPM_TO_JUGGLE);
    startLaunchMotor(m_juggleRpm);
  }

  // This method is called from defaultShooterCmd, or for juggling.
  public void driveFeedMotor(double feedSpeed) {
    m_targetFeedSpeed = feedSpeed;
    m_feedMotor.set(TalonSRXControlMode.PercentOutput, feedSpeed);
  }

  // This method is called to start the feed motor for a launch, or for Juggling.
  public void startFeedMotor(double feedSpeed) {
    if ((Math.abs(feedSpeed) < Shooter.MIN_FEED_SPEED) || (Math.abs(feedSpeed) > Shooter.MAX_FEED_SPEED)) {
      feedSpeed = Shooter.WORKING_FEED_SPEED;
    }
    m_feederStartTimeMs = System.currentTimeMillis();
    System.out.println("Start feed motor at "+feedSpeed+" Percent Out");
    driveFeedMotor(feedSpeed);
  }

  public void stopFeedMotor() {
    // leave m_targetFeedSpeed unchanged, but set output to 0.0 to stop the motor (it 
    // should enter neutral mode, and be in coast)
    m_feedMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

   public void driveLaunchMotor(double rpm) {
    if (rpm <= 0.0) {
      rpm = 0.0;
      stopLaunchMotor();
      return;
    } 
    m_targetLaunchRpm = rpm;
    m_launchMotor.setControl(m_velocityVoltage.withVelocity(Utils.convertRpmToRps(rpm)));
  }
      
  public void startLaunchMotor(double rpm) {
    m_launcherStartTimeMs = System.currentTimeMillis();
    System.out.println("Starting Launch Motor at "+rpm+" rpm");
    driveLaunchMotor(rpm);
  }

  public void stopLaunchMotor() {
    m_targetLaunchRpm = 0.0;
    m_launchMotor.setControl(m_brake);
    // force IDLE if not already there.
    if (m_shooterState != Shooter.IDLE) {
      changeState(Shooter.IDLE);
    }
  }

  // Utility routine to stop motors manually (and return to IDLE state)
  // Called on an X button press, or on detection of an invalid state. 
  public void stopMotors() {
    m_continuousFlag = false;
    stopFeedMotor();
    stopLaunchMotor();
  }

  @Override
  public void periodic() {
    m_measuredLaunchRpm = getLaunchMotorRpm();
    m_avgMeasuredRpm = m_measuredLaunchRpm * 0.2 + m_avgMeasuredRpm * 0.8;
    m_statorLaunchCurrentDirect = m_launchMotor.getStatorCurrent().getValueAsDouble();
    m_supplyLaunchCurrentDirect = m_launchMotor.getSupplyCurrent().getValueAsDouble();
    m_statorLaunchCurrentIndirect = m_launchStatorStatusSignal.refresh().getValueAsDouble();
    m_supplyLaunchCurrentIndirect = m_launchSupplyStatusSignal.refresh().getValueAsDouble();
    m_pdpLaunchSupplyCurrent = m_pdp.getCurrent(Shooter.LAUNCH_MOTOR_PDP_SLOT);
    
    m_timeMs = System.currentTimeMillis();
    m_elapsedSpinUpTimeMs = m_timeMs - m_launcherStartTimeMs;
    m_elapsedLaunchTimeMs = m_timeMs - m_feederStartTimeMs;

    /** Call log method every loop to keep Dashboard data current */
    log();

    switch (m_shooterState) {
      case Shooter.IDLE:
      case Shooter.READY_TO_LAUNCH:
        // Nothing to do.
        break;

      case Shooter.CHANGING_RPM_TO_LAUNCH:
      case Shooter.CHANGING_RPM_TO_JUGGLE:
        if ((Math.abs(m_measuredLaunchRpm - m_targetLaunchRpm) < 50) ||
            (m_elapsedSpinUpTimeMs > Shooter.CHANGE_RPM_TIME_LIMIT)) {
          SmartDashboard.putNumber("MS to reach RPM = ", m_elapsedSpinUpTimeMs);
          if (m_shooterState == Shooter.CHANGING_RPM_TO_LAUNCH) {
            SmartDashboard.putNumber("Starting FeedMotor for launch", m_targetFeedSpeed);
            changeState(Shooter.LAUNCHING);
            m_avgMeasuredRpm = m_measuredLaunchRpm;
            m_feederStartTimeMs = m_timeMs;
            m_supplyLaunchCurrentDirectBaseline = m_supplyLaunchCurrentDirect;
            startFeedMotor(m_targetFeedSpeed);
          } else {
            changeState(Shooter.JUGGLING);
            startFeedMotor(Shooter.JUGGLE_FEED_SPEED);
          }
        }
        break;

      case Shooter.LAUNCHING:
        if (m_continuousFlag) {
          break;
        } 
        System.out.println("Current RPM ="+m_measuredLaunchRpm);
        if ((m_avgMeasuredRpm - m_measuredLaunchRpm) >= (m_targetLaunchRpm*0.03)) {
          //if ((m_supplyLaunchCurrentDirect - m_supplyLaunchCurrentDirectBaseline) >= Shooter.LAUNCH_DETECTION_CURRENT_THRESHOLD) {
          // A spike in supply current means the ball has encountered the launch wheels
          stopFeedMotor();  // the feed motor can be restarted for repetitive shots 
                            // as an option since the shooter motor is left at the last RPM.
          System.out.println("RPM spike detected on launch");
          changeState(Shooter.READY_TO_LAUNCH);
        } else if (m_elapsedLaunchTimeMs > Shooter.LAUNCH_TIME_LIMIT) {
          // timeout, so stop the feed motor and return to READY_TO_LAUNCH state
          stopFeedMotor();
          System.out.println("Ball Launch timed out");
          changeState(Shooter.READY_TO_LAUNCH);
        }
        break;

      case Shooter.JUGGLING:
        driveFeedMotor(Shooter.WORKING_FEED_SPEED);
        break;

      default:
        SmartDashboard.putNumber("Invalid Shooter State detected: ", m_shooterState);
        stopMotors();
        break;
    }
  }

  // The log method puts interesting information to the SmartDashboard.
  public void log() {
    SmartDashboard.putNumber("Var Launch RPM: ", m_variableLaunchRpm);
    SmartDashboard.putNumber("Var Juggle RPM: ", m_juggleRpm);
    SmartDashboard.putNumber("Target RPM: ", m_targetLaunchRpm);
    SmartDashboard.putNumber("Current RPM: ", m_measuredLaunchRpm);
    SmartDashboard.putNumber("Supply Amps Direct: ", m_supplyLaunchCurrentDirect);
    SmartDashboard.putNumber("Supply Amps Indirect: ", m_supplyLaunchCurrentIndirect);
    SmartDashboard.putNumber("Stator Amps Direct: ", m_statorLaunchCurrentDirect);
    SmartDashboard.putNumber("Stator Amps Indirect: ", m_statorLaunchCurrentIndirect);
    SmartDashboard.putNumber("PDP Supply Amps: ", m_pdpLaunchSupplyCurrent);
    SmartDashboard.putNumber("Variable Feed: ", m_variableFeedSpeed);
    SmartDashboard.putNumber("Target Feed: ", m_targetFeedSpeed);
  }
}