// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/** Add your docs here. */
public class Constants {
    public static class DiffDrive {
        public static final int LEFT_MOTOR_CTRL_CAN_ID = 11;
        public static final int LEFT_DRIVE_MOTOR_PDP_SLOT = 2;      // Slot 2 on PDH, 40A fuse
        public static final int RIGHT_MOTOR_CTRL_CAN_ID = 12;
        public static final int RIGHT_DRIVE_MOTOR_PDP_SLOT = 3;     // Slot 3 on PDH, 40A fuse
        public static final double DRIVE_OUTPUT_LIMIT = 0.8;        // Differential drive uses Percent output 
                                                                    // open loop control with manual joystick input

        public static final SupplyCurrentLimitConfiguration driveSupplyCurrentLimit = 
            new SupplyCurrentLimitConfiguration(
                true,                      // driveEnableCurrentLimit, 
                40,                  // driveContinuousCurrentLimit,
                60,       // drivePeakCurrentLimit, 
                0.3);        // drivePeakCurrentDuration);
    }

    public static class Shooter {
        // Feed motor is a BAG controlled with a TalonSRX through a 
        public static final int FEED_MOTOR_CTRL_CAN_ID = 9;             // Uses Phoenix5 Percent output control,  
                                                                        // with both program and joystick input
        public static final int FEED_MOTOR_PDP_SLOT = 15;               // Slot 15 on PDH, 40A Fuse,                                                             
        public static final SupplyCurrentLimitConfiguration feedSupplyLimit = 
                                new SupplyCurrentLimitConfiguration(
                                    true,                      // feedEnableCurrentLimit, 
                                    20,                  // feedContinuousCurrentLimit,
                                    40,       // feedPeakCurrentLimit, 
                                    0.1);        // feedPeakCurrentDuration);

        public static final double MIN_FEED_SPEED = 0.3;
        public static final double WORKING_FEED_SPEED = 0.7;
        public static final double JUGGLE_FEED_SPEED = 0.65;
        public static final double MAX_FEED_SPEED = 1.0;

        public static final int    LAUNCH_MOTOR_TALON_FX_CAN_ID = 10;
        public static final int    LAUNCH_MOTOR_PDP_SLOT = 1;               // Slot 1 on PDH, 40A fuse
                                                                            // Phoenix6 closed loop Velocity PID control, 
                                                                            // normally with fixed input, but manual joystick 
                                                                            // scaled input avail for variable operator control
        public static final int    LAUNCH_PID_SLOT = 0;
        // The following PID constants (assigned here to SLOT0) are given in 
        // the standard Github CTRE VelocityClosedLoop example:
        public static final double KP = 0.2;            // .11; error of 1 rps results in 2 V output
        public static final double KI = 0.4;            // 0.5; error of 1 rps increases output by 0.5 V every second
        public static final double KD = 0.0001;         // .0001; change of 1 rotation / (sec squared) results in 0.01 V output
        public static final double KS = 0.0;
        public static final double KV = 0.12;           // This is the old "feed forward" replacement
                                                        // Falcon 500 is a 500kV motor; 500 rpm/volt = 8.333 rps/v, or 0.12V/rps
                                                        // Max speed of 6380 RPM or 106.33 rps results from a calculated input 
                                                        // of 12.76 volts
        public static final double KA = 0.0;
        public static final double KG = 0.0;
        // kV in Phoenix6 has replaced kF in phoenix5. Was 0.049516;

        public static final double LAUNCH_DETECTION_CURRENT_THRESHOLD = 10.0;            // amps

        public static final double MAX_FALCON500_RPM = 6380;
        public static final double MIN_LAUNCH_RPM = 400;               // Use for Juggle; FF ~= .2 proportional output
        public static final double MAX_LAUNCH_RPM = 4075;               // FF ~= .65  "
        public static final double WORKING_LAUNCH_RPM = 3700;          // FF ~= .55  " for 3425 RPM
        public static final double JUGGLE_LAUNCH_RPM = 550.0;

        public static final double CHANGE_RPM_TIME_LIMIT = 2500.0;      // Launch motor spin up timeout
        public static final double LAUNCH_TIME_LIMIT = 2000.0;          // Feed motor timeout - for stopping feed motor (if 
                                                                        // sensing a launch does not stop it first.
                                                                        // Sensing is done via power monitoring of launch motor)
 
         // Possible States (could be bundled into an enum):
        public static final int IDLE                    = 0;
        public static final int CHANGING_RPM_TO_LAUNCH  = 1;
        public static final int READY_TO_LAUNCH         = 2; 
        public static final int LAUNCHING               = 3;
        public static final int CHANGING_RPM_TO_JUGGLE  = 4;
        public static final int JUGGLING                = 5;
    }

    public class UI {
        public static final double JOYSTICK_DEADBAND = .1;      // For Driving and shooter ball Feed
        public static final double TRIGGER_DEADBAND = .05;      // for flywheel
    }
}
