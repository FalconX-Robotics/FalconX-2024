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
  public static final double SLIP_FACTOR = 1.;
  public static final double KITBOT_WHEEL_DIAMETER = 4. * SLIP_FACTOR;
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
  }
  public static class RatioConstants {
    public static final double KITBOT_GEAR_RATIO = 50.0/14.0 * 40.0/24.0;
    public static final double NESSIE_GEAR_RATIO = 50.0/12.0 * 50.0/24.0;
    public static final double ArmGearRatio = 120.;
    // TODO: set to a reasonable value
  }
  /** To be changed later; change values based on the weight, offset, etc of the arm */
  public static class ArmFeedForwardConstants {
    public static final double maxVelocity = 100;
    public static final double maxAcceleration = 4.;

    /** In radians, the offset of the arm */
    public static final double offset = Math.toRadians(25);
    public static final double staticGain = 0.2;
    public static final double gravityGain = 0.5;
    public static final double velocityGain = 12/((5600. / RatioConstants.ArmGearRatio / 60.) * (2. * Math.PI));
  }
  public static class MotorConstants {
    // 0 is reserved for RoboRIO and just generally shouldn't be used
    // 1 is reserved for Rev PDP

    public static final boolean donatello = true; // :)

    //TODO: This probably won't work. test and remove if not. -w
    public enum DrivetrainMotors {
      frontLeft  (5),
      frontRight (2, 9),
      backLeft   (3),
      backRight  (4);

      public int value;
      private DrivetrainMotors(int donatelloValue, int nessieValue){
        value = donatello ? donatelloValue:nessieValue;
      }
      private DrivetrainMotors(int bothValue){
        value = bothValue;
      }
    }
    public static final int index = 10;
    public static final int intake = 11;
    public static final int shooter = 12;
    public static final int shooterFollower = 13;
    public static final int arm = 20;
    public static final int armFollower = 21;
    public static final int climber = 30;
    public static final int climberFollower = 31;
  }

  public static class VisionConstants {
    public static final double CAMERA_HEIGHT_METERS = 0.0;
    public static final double TARGET_HEIGHT_METERS = 0.0;
    public static final double CAMERA_PITCH_RADIANS = 0.0;
  }

  public static final int gyroId = 42;
}
