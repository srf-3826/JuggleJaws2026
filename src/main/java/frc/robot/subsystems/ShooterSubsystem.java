// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants.*;
import frc.robot.Utils.Utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The ball shooter subsystem uses PID to go to a given RPM, and when the setpoint
 * is reached, it starts the feed motor to drive the ball into the shooter wheels.
 */
public class ShooterSubsystem extends SubsystemBase {
  private TalonFX       m_launchMotor;
  private double        m_variableLaunchRpm = Shooter.WORKING_LAUNCH_RPM;     // default. Can be adjusted via Dpad up or down
  private double        m_measuredLaunchRpm;
  private double        m_targetLaunchRpm = 0.0;       // Most recent requested launch speed
  private double        m_juggleRpm = Shooter.JUGGLE_LAUNCH_RPM;              // default. Can be adjusted via DPad in JUGGLING state.
  private double        m_statorLaunchCurrent;
  private double        m_supplyLaunchCurrent;
  private double        m_supplyLaunchCurrentBaseline = 0.0;
  private double        m_pdpLaunchSupplyCurrent;

  private TalonSRX      m_feedMotor;
  private double        m_targetFeedSpeed = 0.0;
  private double        m_variableFeedSpeed = Shooter.WORKING_FEED_SPEED;
  private double        m_juggleFeedSpeed = Shooter.JUGGLE_FEED_SPEED;    // Default, Can be adjusted in JUGGLING state
  private double        m_avgMeasuredRpm = 0.0; 
 
  private boolean       m_continuousFlag = false;

  // Phoenix6 control mode for Launch motor
  private VelocityVoltage m_velocityVoltageRequest = new VelocityVoltage(0);
 
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

  private final CANBus            m_canBus;
  private final PowerDistribution m_pdp;

  //=============================
  // ShooterSubsystem constructor
  //=============================
  public ShooterSubsystem(CANBus canBus, PowerDistribution pdp) {
    m_canBus = canBus;
    m_pdp = pdp;

    m_feedMotor = new TalonSRX(Shooter.FEED_MOTOR_CTRL_CAN_ID);
    configFeedMotor();
  
    m_launchMotor = new TalonFX(Shooter.LAUNCH_MOTOR_TALON_FX_CAN_ID, m_canBus);
    configLaunchMotor();
  }

  //============================
  // motor configuration methods
  //============================

  // Feed motor is controlled by a TalonFRX (requires Phoenix5)
  private void configFeedMotor() {
    m_feedMotor.configFactoryDefault();
    // m_feedMotor.configAllSettings(ctreConfigs.feedSRXConfig);
    m_feedMotor.configSupplyCurrentLimit(Shooter.feedSupplyLimit);
    m_feedMotor.setInverted(true);
    m_feedMotor.setNeutralMode(NeutralMode.Coast);
  } 

  // Launch motor is a TaloxFX, which uses Phoenix6
  private void configLaunchMotor() {
    var launchOutputConfig = new MotorOutputConfigs().withNeutralMode(Shooter.LAUNCH_MOTOR_NEUTRAL_MODE)
                                                     .withInverted(Shooter.LAUNCH_MOTOR_INVERT)
                                                     .withPeakForwardDutyCycle(Shooter.LAUNCH_PEAK_DUTY_CYCLE)
                                                     .withPeakReverseDutyCycle(-Shooter.LAUNCH_PEAK_DUTY_CYCLE);
    var voltageConfig = new VoltageConfigs().withPeakForwardVoltage(Shooter.LAUNCH_PEAK_VOLTAGE)
                                            .withPeakReverseVoltage(-Shooter.LAUNCH_PEAK_VOLTAGE);
    var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                              .withFeedbackRemoteSensorID(0)
                                              .withSensorToMechanismRatio(Shooter.SENSOR_TO_MECHANISM_RATIO)
                                              .withRotorToSensorRatio(Shooter.ROTOR_TO_SENSOR_RATIO);
    var pid0Config = new Slot0Configs().withKP(Shooter.KP)
                                       .withKI(Shooter.KI)
                                       .withKD(Shooter.KD)
                                       .withKS(Shooter.KS)
                                       .withKV(Shooter.KV)
                                       .withKA(Shooter.KA)
                                       .withKG(Shooter.KG);
    var openLoopRampConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(Shooter.LAUNCH_OPEN_LOOP_RAMP_PERIOD)
                                                       .withVoltageOpenLoopRampPeriod(Shooter.LAUNCH_OPEN_LOOP_RAMP_PERIOD)
                                                       .withTorqueOpenLoopRampPeriod(0);
    var closedLoopRampConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Shooter.LAUNCH_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withVoltageClosedLoopRampPeriod(Shooter.LAUNCH_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(Shooter.LAUNCH_SUPPLY_CURRENT_LIMIT)
                                                       .withSupplyCurrentLimitEnable(Shooter.LAUNCH_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                       .withStatorCurrentLimit(Shooter.LAUNCH_STATOR_CURRENT_LIMIT)
                                                       .withStatorCurrentLimitEnable(Shooter.LAUNCH_ENABLE_STATOR_CURRENT_LIMIT);
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

  //========================
  // ShooterState management
  //========================
  // changeState is a Utility routine that changes state, but also ensures a record is kept
  // showing all changes, including the immediately previous state.
  public void changeState(int newState) {
    SmartDashboard.putNumber("Shooter State = ", newState);
    System.out.println("Change State from "+m_shooterState+" to "+newState);
    m_shooterState = newState;
  }

  //===================================================
  // Routines to adjust variable Launch and Feed speeds
  // Called via button bindings only
  //===================================================
  
  // The following two methods allow on-the-fly adjustment of the launch speed.
  //
  // For regular lauching, adjust in 200 RPM increments, limiting the result
  // to be between MIN_LAUNCH_RPM and MAX_LAUNCH_RPM, and do it wihtout any
  // active motor control, i.e. defer that to the next Launch control button 
  // (A, Y, Alt-A, or ALT-Y) press.
  //
  // For Juggling, adjust the currently active Launch RPM by just 100 and apply the 
  // change immediately, and limit the max RPM to JUGGLE_LAUNCH_RPM, and the min RPM to 
  // (JUGGLE_LAUNCH_RPM - 300). This allow fine tuning of the Juggle to reduce chances
  // of the ball overshooting the basket during parades or other robot movements.
  
  public void decreaseVariableLaunchRpm() {
    if (m_shooterState == Shooter.JUGGLING) {
      if (m_juggleRpm >= (Shooter.JUGGLE_LAUNCH_RPM-200.0)) {
        m_juggleRpm -= 100.0;
        driveLaunchMotor(m_juggleRpm);
      }
    } else if (m_variableLaunchRpm >= (Shooter.MIN_LAUNCH_RPM + 200.0)) {
      m_variableLaunchRpm -= 200.0;
    }
  }

  public void increaseVariableLaunchRpm() {
    if (m_shooterState == Shooter.JUGGLING) {
      if (m_juggleRpm <= (Shooter.JUGGLE_LAUNCH_RPM - 100)) {
        m_juggleRpm += 100.0;
        driveLaunchMotor(m_juggleRpm);
      }
    } else if (m_variableLaunchRpm <= Shooter.MAX_LAUNCH_RPM - 200.0) {
      m_variableLaunchRpm += 200.0;
    }
  }

  // The following two methods adjust the variableFeedSpeed in increments of
  // .2, keeping the speed within the range MIN_FEED_SPEED (.3) and MAX_FEED_SPEEED (1.0),
  // OR, if in JUGGLING state, adjust the m_jugglingFeedSpeed in increments of
  // 0.1, keeping within the range JUGGLE_FEED_SPEED - .3 and JUGGLE_FEED_SPEED + 0.1

  public void decreaseVariableFeedSpeed() {
    if (m_shooterState == Shooter.JUGGLING) {
      if (m_juggleFeedSpeed >= Shooter.JUGGLE_FEED_SPEED - 0.2) {
        m_juggleFeedSpeed -= 0.1;
      }
    } else if (m_variableFeedSpeed >= (Shooter.MIN_FEED_SPEED + 0.2)) {
      m_variableFeedSpeed -= 0.2;
    }
  }

  public void increaseVariableFeedSpeed() {
    if (m_shooterState == Shooter.JUGGLING) {
      if (m_juggleFeedSpeed <= Shooter.JUGGLE_FEED_SPEED) {
        m_juggleFeedSpeed += 0.1;
      }
    } else if (m_variableFeedSpeed <= (Shooter.MAX_FEED_SPEED - 0.2)) {
      m_variableFeedSpeed += 0.2;
    }
  }

  public void resetAllSpeeds() {
    m_variableLaunchRpm = Shooter.WORKING_LAUNCH_RPM;
    m_variableFeedSpeed = Shooter.WORKING_FEED_SPEED;
    m_juggleRpm = Shooter.JUGGLE_LAUNCH_RPM;
    m_juggleFeedSpeed = Shooter.JUGGLE_FEED_SPEED;
  }
  
  //======================================================================================
  // Getters for targetLaunchSpeed, targeetFeedSpeed, Shooter State, and Shooter motor RPM
  //======================================================================================

  public double getTargetLaunchSpeed() {
    return m_targetLaunchRpm;
  }

  public double getTargetFeedSpeed() {
    return m_targetFeedSpeed;
  }

  public int getCurrentState() {
    return m_shooterState;
  }

  // getLaunchMotorRpm returns the current RPM of the launch motor.
  // It reads the internal encoder (which in Phoenix6 returns units rps)
  // and converts that to RPM, which is the value returned
  public double getLaunchMotorRpm() {
    return Utils.convertRpsToRpm(m_launchMotor.getVelocity().getValueAsDouble());
  }

  //================================================================
  // methods to support buttom presses which control launching balls
  //================================================================

  // A WORD ABOUT SPEEDS AND PROGRAM LOGIC:
  // There are essentially 4 speeds stored or tracked:
  // WORKING_LAUNCH_RPM and WORKING_FEED_SPEED are pre-set constant speeds
  // JUGGLE_LAUNCH_RPM and JUGGLE_FEED_SPEED and also pre-set constants speeds
  // but that only apply during juggling.
  // Both of these seed variables holding adjustable speeds, for which
  // m_variableLaunchRpm and m_variableFeedSpeed are bound to the Y button for
  // lauching balls at those variable speeds, and the fixed WORKING_LAUNCH_RPM and 
  // WORKING_FEED_SPEED are bound to the A button for launching balls at fixed speeds.
  // For juggling, the (possibly adjusted) m_juggleRpm and m_juggleFeedSpeed are
  // always using when entering the JUGGLE State.
  //
  // m_targetLaunchRpm and m_targetFeedSpeed, regardless of State, are stored speeds 
  // last set by any motor controller directive. For deferred ball launch actions 
  // (waiting for shooter to come to speed, for example) it is necessary that 
  // m_targetFeedSpeed be set beforehand (this is usually not set until the motor "drive" 
  // method is called), so that the state maachine when ready can start the feed motor at 
  // the desired speed (either FIXED or variable). 
  // The point being that the state machine will always use the m_targetxxx speeds when 
  // ball launching is ready. Juggling will always use its own speeds when ready.
  // Also, the current target speeds (regardless of state) are always available to be stored
  // before any defaultShooterCmd overrides are applied. Then they can be restored (by calling 
  // the respective motor "drive" method with the stored speeds) whenever such overrides are 
  // no longer present.
  
  public void launchBallVariableRpm(boolean continuousFlag) {                    // a or ALT-a (ALT = continuous)
    launchBall(m_variableLaunchRpm, m_variableFeedSpeed, continuousFlag);
  }

  public void launchBallFixedRpm(boolean continuousFlag) {                       // y or ALT-y (ALT = continuous)
    launchBall(Shooter.WORKING_LAUNCH_RPM, Shooter.WORKING_FEED_SPEED, continuousFlag);
  }

  // launchBall is the central method to initiate ball launching, either with a timeout to limit 
  // the number of balls launched, or alternatively using current sensing to stop after launching
  // just one ball) or continuous (no limits on how many balls to launch, not stopping until
  // and appropriate button press (i.e., X)).
  // It is allowed to be called in any state, even JUGGLING or CHANGING_RPM, as well as LAUNCHING 
  // (because launch and feed speeds are allowed to change), but it is only called from 
  // one of the nmethods just above, either launchBallVariableRpm or launchBallFixedRpm. 
  // This method sets the appropriate state and sends motor controller closed loop directives
  // to change speed(s). The periodic() method must take over while waiting for the launch motor
  // target RPM to be reached, after which the feed motor will be started to move ball(s) into 
  // contact with the shooter wheels.
  // periodic() will also stop the feed motor when it is desired to shoot just one ball (or a limited 
  // number of balls) based on the consumed amperage (shooter moter supply or stator, or PDP slot, any or
  // all), or via a timeot. If the continuousFlag is set, the launching will run until canceled.
  //
  private void launchBall(double launchRpm, double feedSpeed, boolean continuousFlag) {
    m_continuousFlag = continuousFlag;
    if ((m_shooterState == Shooter.LAUNCHING) || (m_shooterState == Shooter.READY_TO_LAUNCH)) {
      startFeedMotor(feedSpeed);
      changeState(Shooter.LAUNCHING);
      return;
    }
    stopFeedMotor();
    m_targetFeedSpeed = feedSpeed;
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

  // This method is called from defaultShooterCmd, or from startFeedMotor,
  // or per loop while juggling.
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

  // driveLaunchMotor starts the launch motor with no timeout (if timeout needed, set it
  // before or after calling this method). If the specified rpm is 0 or negative,
  // stop the motor. Always set the targetLaunchRpm to the requested speed.
   public void driveLaunchMotor(double rpm) {
    m_targetLaunchRpm = rpm;
    if (rpm <= 0.0) {
      m_targetLaunchRpm = 0.0;
      m_launchMotor.stopMotor();
      if (m_shooterState != Shooter.IDLE) {
        changeState(Shooter.IDLE);
      }
    } else {
      m_launchMotor.setControl(m_velocityVoltageRequest.withVelocity(Utils.convertRpmToRps(rpm)));
    }
  }

  // startLaunchMotor essentially replicates driveLaunchMotor except it also sets a start time
  // and reports the requested rpm speed.
  public void startLaunchMotor(double rpm) {
    m_launcherStartTimeMs = System.currentTimeMillis();
    System.out.println("Starting Launch Motor at "+rpm+" rpm");
    driveLaunchMotor(rpm);
  }

  public void stopLaunchMotor() {
    driveLaunchMotor(0.0);
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
    m_statorLaunchCurrent = m_launchMotor.getStatorCurrent().getValueAsDouble();
    m_supplyLaunchCurrent = m_launchMotor.getSupplyCurrent().getValueAsDouble();
    m_pdpLaunchSupplyCurrent = m_pdp.getCurrent(Shooter.LAUNCH_MOTOR_PDP_SLOT);
    
    m_timeMs = System.currentTimeMillis();
    m_elapsedSpinUpTimeMs = m_timeMs - m_launcherStartTimeMs;
    m_elapsedLaunchTimeMs = m_timeMs - m_feederStartTimeMs;

    /** Call log method every loop to keep Dashboard data current */
    publishShooterData();

    // Run the shooter state machine
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
            changeState(Shooter.LAUNCHING);
            // reset baselines to current values at time lauch is triggered
            m_avgMeasuredRpm = m_measuredLaunchRpm;
            m_supplyLaunchCurrentBaseline = m_supplyLaunchCurrent;
            SmartDashboard.putNumber("Starting FeedMotor for launch", m_targetFeedSpeed);
            startFeedMotor(m_targetFeedSpeed);
          } else {
            changeState(Shooter.JUGGLING);
            startFeedMotor(m_juggleFeedSpeed);
          }
        }
        break;

      case Shooter.LAUNCHING:
        if (m_continuousFlag) {
          break;
        }
        // Not in continuous launch mode, so check if feeder (and maybe launcher) should be stopped
        System.out.println("Current RPM = "+m_measuredLaunchRpm);
        if ((m_avgMeasuredRpm - m_measuredLaunchRpm) >= (m_targetLaunchRpm*0.03)) {
          if ((m_supplyLaunchCurrent - m_supplyLaunchCurrentBaseline) >= Shooter.LAUNCH_DETECTION_CURRENT_THRESHOLD) {
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
        }
        break;

      case Shooter.JUGGLING:
        driveFeedMotor(m_juggleFeedSpeed);
        break;

      default:
        SmartDashboard.putNumber("Invalid Shooter State detected: ", m_shooterState);
        stopMotors();
        changeState(Shooter.IDLE);
        break;
    }
  }

  // The log method puts interesting information to the SmartDashboard.
  public void publishShooterData() {
    SmartDashboard.putNumber("Var Launch RPM: ", m_variableLaunchRpm);
    SmartDashboard.putNumber("Var Feed Speed: ", m_variableFeedSpeed);
    SmartDashboard.putNumber("Var Juggle RPM: ", m_juggleRpm);
    SmartDashboard.putNumber("Var Juggle Feed Speed: ", m_juggleFeedSpeed);
    SmartDashboard.putNumber("Target RPM: ", m_targetLaunchRpm);
    SmartDashboard.putNumber("Target Feed speed: ", m_targetFeedSpeed);
    SmartDashboard.putNumber("Measured RPM: ", m_measuredLaunchRpm);
    SmartDashboard.putNumber("Supply Amps: ", m_supplyLaunchCurrent);
    SmartDashboard.putNumber("Stator Amps: ", m_statorLaunchCurrent);
    SmartDashboard.putNumber("PDP Supply Amps: ", m_pdpLaunchSupplyCurrent);


  }
}