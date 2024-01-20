// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final double KITBOT_GEAR_RATIO = 8.45;
  public static final double NESSIE_GEAR_RATIO = 50.0/12.0 * 50.0/24.0;

  //FIX THIS WHEN PIGEON IS ADDED
  public static final int PIGEON_PORT = 0;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kShooterControllerPort = 1;
  }
  public static class MotorConstants {
    public static final int frontLeft = 0;
    public static final int frontRight = 0;
    public static final int backLeft = 0;
    public static final int backRight = 0;

    public static final int bottomIntake = 0;
    public static final int topIntake = 0;
    public static final int shooter = 0;
    public static final int shooterArm = 0;
  }
  public static final int gyroId = 42;
}
