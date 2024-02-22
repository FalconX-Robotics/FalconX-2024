package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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

  DataLog log = DataLogManager.getLog();
  DoubleLogEntry gyroEntry = new DoubleLogEntry(log, "/odometry/gyroAngleDegrees");
  DoubleLogEntry leftEncoderPositionEntry = new DoubleLogEntry(log, "/odometry/leftPosition");
  DoubleLogEntry rightEncoderPositionEntry = new DoubleLogEntry(log, "/odometry/rightPosition");
  DoubleLogEntry leftEncoderVelocityEntry = new DoubleLogEntry(log, "/odometry/leftVelocity");
  DoubleLogEntry rightEncoderVelocityEntry = new DoubleLogEntry(log, "/odometry/rightVelocity"); 
  DoubleLogEntry targetX = new DoubleLogEntry(log, "/odometry/targetX");
  DoubleLogEntry targetY = new DoubleLogEntry(log, "/odometry/targetY");
  DoubleLogEntry targetRotationDegrees = new DoubleLogEntry(log, "/odometry/targetRotationDegrees");
  

  PIDController leftController = new PIDController(0., 0.0, 0.0);
  PIDController rightController = new PIDController(0., 0.0, 0.); //this sorta works (maybe? (i dont know))
  
  private static final double   kTrackWidth = 0.457; // meters, this is the defauklt from wpilib
                                                       // change this later

  private final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(kTrackWidth);

    
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.52, 2.4, 0.7);
    
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

        resetPose(new Pose2d(1, 7, new Rotation2d()));
        
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
          targetX.append(pose.getX());
          targetY.append(pose.getY());
          targetRotationDegrees.append(pose.getRotation().getDegrees());
        });
    }

  DifferentialDriveOdometry m_odometry;

  public void periodic() {
    pose = m_odometry.update(getRotation(),
    m_leftEncoder.getPosition(),
    m_rightEncoder.getPosition());

    field2d.setRobotPose(pose);

    SmartDashboard.putNumber("Left Encoder Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Position", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Encoder Velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Encoder Velocity", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("gyro", getRotation().getDegrees());

    gyroEntry.append(getRotation().getDegrees());
    leftEncoderPositionEntry.append(m_leftEncoder.getPosition());
    rightEncoderPositionEntry.append(m_rightEncoder.getPosition());
    leftEncoderVelocityEntry.append(m_leftEncoder.getVelocity());
    rightEncoderVelocityEntry.append(m_rightEncoder.getVelocity());

  }

  public void resetPose(Pose2d newPose) {
    gyro.reset();
    REVLibError errorLeft = m_leftEncoder.setPosition(0);
    REVLibError errorRight = m_rightEncoder.setPosition(0);

    SmartDashboard.putString("Left errror", errorLeft.toString());
    SmartDashboard.putString("Right errror", errorRight.toString());

    m_odometry = new DifferentialDriveOdometry(
          new Rotation2d(),
          0, 0,
          newPose);
    pose = m_odometry.getPoseMeters();



    SmartDashboard.putNumber("Rotation via pose at reset", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Left Encoder Position at reset", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Position at reset", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Pose x at reset", pose.getX());
    SmartDashboard.putNumber("Pose y at reset", pose.getY());
    SmartDashboard.putNumber("Gyro rotation at reset", getRotation().getDegrees());

    if (Robot.isSimulation()) {
      drivetrain.getSimulation().setPose(pose);
      // set enocder positions/velocity to 0
      drivetrain.getSimulation().setState(new Matrix<>(Nat.N7(), Nat.N1()));
    }
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
    SmartDashboard.putNumber("left m/s", speeds.leftMetersPerSecond);
    SmartDashboard.putNumber("right m/s", speeds.rightMetersPerSecond);
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
    // final double leftFeedforward = 0;
    // final double rightFeedforward = 0;
    final double leftOutput =
        leftController.calculate(m_leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightController.calculate(m_rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
    SmartDashboard.putNumber("right pid output", rightOutput);
    SmartDashboard.putNumber("left pid output", leftOutput);
    
    drivetrain.setMotorVoltage(leftOutput + leftFeedforward, rightOutput + rightFeedforward);    
  }

  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setSpeeds(kinematics.toWheelSpeeds(chassisSpeeds));
  }

  private Rotation2d getRotation() {
    if (Robot.isSimulation()) return simulationGyro;

    return gyro.getRotation2d();
  }
  //kitbot 4 inches wheels
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
