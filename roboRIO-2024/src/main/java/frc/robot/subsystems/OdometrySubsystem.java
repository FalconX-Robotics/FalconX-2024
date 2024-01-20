package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.simulation.RelativeEncoderSim;

public class OdometrySubsystem {


  Pose2d pose = new Pose2d();

  WPI_PigeonIMU gyro;
  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;
  Drivetrain drivetrain;
    
  Field2d field2d = new Field2d();

  PIDController leftController = new PIDController(0.2, 0, 0.02);
  PIDController rightController = new PIDController(0.2, 0, 0.02);

  private static final double kTrackWidth = 0.381 * 2; // meters, this is the defauklt from wpilib
                                                       // change this later

  private final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(kTrackWidth);

    
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);
    
    // constructor so i can find in in the wall of code
  public OdometrySubsystem (Drivetrain drivetrain) {
      this.drivetrain = drivetrain;
      gyro = drivetrain.getGyro();
      m_leftEncoder = drivetrain.getLeftEncoder();
      m_rightEncoder = drivetrain.getRightEncoder();

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
        drivetrain // Reference to this subsystem to set requirements
        );

        if (Robot.isSimulation()) {
          m_leftEncoder = new RelativeEncoderSim();
          m_rightEncoder = new RelativeEncoderSim();
        }
        m_odometry = new DifferentialDriveOdometry(
          getRotation(),
          m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
          new Pose2d(1.94, 3.79, new Rotation2d()));
    }

  DifferentialDriveOdometry m_odometry;

  public void  periodic() {
    pose = m_odometry.update(getRotation(),
    m_leftEncoder.getPosition(),
    m_rightEncoder.getPosition());

    field2d.setRobotPose(pose);
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
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    return kinematics.toChassisSpeeds(wheelSpeeds);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
    final double leftOutput =
        leftController.calculate(m_leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightController.calculate(m_rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
    drivetrain.setMotorVoltage(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }

  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setSpeeds(kinematics.toWheelSpeeds(chassisSpeeds));
  }

  private Rotation2d getRotation() {
    if (Robot.isSimulation()) return simulationGyro;

    return gyro.getRotation2d();
  }
  //4 inches wheels
  //nessie 6 inches wheels


  
  Rotation2d simulationGyro = new Rotation2d();

  public void simulationPeriodic() {
    SmartDashboard.putData(field2d);

    DifferentialDrivetrainSim simulation = drivetrain.getSimulation();
    simulationGyro = simulation.getHeading();

    RelativeEncoderSim leftEncoder = (RelativeEncoderSim) m_leftEncoder;
    RelativeEncoderSim rightEncoder = (RelativeEncoderSim) m_rightEncoder;
    leftEncoder.setSimulationPositionMeters(simulation.getLeftPositionMeters());
    rightEncoder.setSimulationPositionMeters(simulation.getRightPositionMeters());
    leftEncoder.setSimulationVelocityMetersPerSecond(simulation.getLeftVelocityMetersPerSecond());
    rightEncoder.setSimulationVelocityMetersPerSecond(simulation.getRightVelocityMetersPerSecond());
  }
}
