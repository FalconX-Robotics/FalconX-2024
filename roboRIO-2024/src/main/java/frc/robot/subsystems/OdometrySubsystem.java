package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Settings;

/**<img src="https://imageio.forbes.com/specials-images/imageserve/62bf4a76be9035384b5bfc2e/The-Dad-Gang-Founder-and-Author-Sean-Williams/960x0.jpg?format=jpg&width=1440"> **/
public class OdometrySubsystem extends SubsystemBase{


  Pose2d pose = new Pose2d();

  WPI_PigeonIMU gyro;
  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  Drivetrain drivetrain;
    
  Field2d field2d = new Field2d();

  PIDController leftController = new PIDController(0.2, 0, 0.02);
  PIDController rightController = new PIDController(0.2, 0, 0.02);

  private static final double kTrackWidth = 0.381 * 2; // meters, this is the defauklt from wpilib
                                                       // change this later

  private final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(kTrackWidth);

    
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);
    
    // constructor so i can find in in the wall of code
  public OdometrySubsystem (Drivetrain drivetrain) {
      this.drivetrain = drivetrain;
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
    getRotation(),
    leftEncoder.getPosition(), rightEncoder.getPosition(),
    new Pose2d(5.0, 13.5, new Rotation2d()));

  @Override
  public void periodic() {
    pose = m_odometry.update(getRotation(),
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
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    return kinematics.toChassisSpeeds(wheelSpeeds);
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

  private Rotation2d getRotation() {
    return gyro.getRotation2d();
  }

  private double getAngularVelocity() {
    return gyro.getRate();
  }
  //4 inches
  //6 inches nessie
}
