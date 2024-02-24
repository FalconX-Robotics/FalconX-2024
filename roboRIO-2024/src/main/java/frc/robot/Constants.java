// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double KITBOT_WHEEL_DIAMETER = 4.;
  public static final double NESSIE_WHEEL_DIAMETER = 6.;

  //FIX THIS WHEN PIGEON IS ADDED
  public static final int PIGEON_PORT = 42;

  // LED is in PWM and thus doesn't need a unique value unless there are other things in PWM.
  public static final int LED_PORT = 0;

  // Sensor is in DIO and thus doesn't need a unique value unless there are other things in DIO..
  public static final int SENSOR_PORT = 9;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kShooterControllerPort = 1;

    public static final double INPUT_CURVE_AMOUNT = 2;
  }
  public static class RatioConstants {
    public static final double KITBOT_GEAR_RATIO = 8.45;
    public static final double NESSIE_GEAR_RATIO = 50.0/12.0 * 50.0/24.0;
    public static final double ArmGearRatio = 1.;
    // TODO: set to a reasonable value
  }
  public static class MotorConstants {
    // 0 is reserved for RIO
    // 1 is reserved for PDP

    public static final boolean donatello = true; // :)

    //TODO: This probably won't work. test and remove if not. -w
    public enum drivetrain {
      frontLeft  (5),
      frontRight (2, 9),
      backLeft   (3),
      backRight  (4);

      public int value;
      private drivetrain(int donatelloValue, int nessieValue){
        value = donatello ? donatelloValue:nessieValue;
      }
      private drivetrain(int bothValue){
        value = bothValue;
      }
    }
    public static final int index = 10;
    public static final int intake = 11;
    public static final int shooter = 12;
    public static final int shooterFollower = 13;
    public static final int arm = 20;
    public static final int armFollower = 21;
  }
}
