// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

public class Utils {
    // Talon controllers accept and return vel sensor data as counts per 100 ms.
    // Using the internal encoder, there are 2048 counts per motor revolution.

    // The launch motor has no gearbox or pully reduction, so motor RPM is
    // the same as launch wheels and flywheel RPM.
    // To convert Rotations per second to RPM, just multiply by 60
    public static double convertRpsToRpm( double rps ) {
        return (rps * 60.0);
    }
  
    public static double convertRpmToRps( double rpm ) {
        return (rpm / 60.0);
    }
}
