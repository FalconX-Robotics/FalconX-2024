package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;

/**<h1>i hate this **/
public class Odometry extends SubsystemBase{


    Pose2d pose = new Pose2d();

    PigeonIMU gyro;
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;
    Drivetrain drivetrain = new Drivetrain();
    

    PIDController leftController = new PIDController(0.00001, 0, 1);
    PIDController rightController = new PIDController(0.00001, 0, 1);

    private static final double kTrackWidth = 0.381 * 2; // meters, this is the defauklt from wpilib
                                                         // change this later

    private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

    
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);
    
    // constructor so i can find in in the wall of code
    public Odometry () {
        gyro = drivetrain.getGyro();
        leftEncoder = drivetrain.getLeftEncoder();
        rightEncoder = drivetrain.getRightEncoder();

        AutoBuilder.configureRamsete(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // Current ChassisSpeeds supplier
                this::driveChassisSpeeds, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
  new Rotation2d(BaseUnits.Angle.convertFrom(gyro.getYaw(), Units.Degrees)),
  leftEncoder.getPosition(), rightEncoder.getPosition(),
  new Pose2d(5.0, 13.5, new Rotation2d()));

  @Override
  public void periodic() {
    var gyroAngle = gyro.getYaw();

  // Update the pose
    pose = m_odometry.update(new Rotation2d(gyroAngle),
    leftEncoder.getPosition(),
    rightEncoder.getPosition());
  }
  
  public void resetPose(Pose2d newPose) {
    pose = newPose; 
  }

//   public void resetPose(Pose2d pose) {
//     resetPose(pose.getRotation(), pose.);
//   }

  public Pose2d getPose() {
    return pose;
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return new ChassisSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity(), kTrackWidth);
  }

  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }
  

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        leftController.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightController.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
    drivetrain.setMotors(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }

  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setSpeeds(kinematics.toWheelSpeeds(chassisSpeeds));
  }

  public static boolean flipPaths() {
    return Settings.team == Constants.TeamColors.RED;   
  }
}
