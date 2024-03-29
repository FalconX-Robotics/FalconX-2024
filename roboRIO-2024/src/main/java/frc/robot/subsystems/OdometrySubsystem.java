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
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardHelper;
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
  
  
  PIDController leftController = new PIDController(3,0,0);
  PIDController rightController = new PIDController(3,0,0); //this sorta works (maybe? (i dont know))
  
  private static final double kTrackWidth = Units.Inches.toBaseUnits(22); // meters, this is the defauklt from wpilib
                                                   // change this later

  private final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(kTrackWidth);

    
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.52, 2.4, 0.3);
    
    // constructor so i can find in in the wall of code
  public OdometrySubsystem (Drivetrain drivetrain) {
      leftController.setIntegratorRange(-2, 2);
      rightController.setIntegratorRange(-2,2);
      this.drivetrain = drivetrain;
      gyro = drivetrain.getGyro();
      m_leftEncoder = drivetrain.getLeftEncoder();
      m_rightEncoder = drivetrain.getRightEncoder();

      AutoBuilder.configureRamsete(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentSpeeds, // Current ChassisSpeeds supplier
        this::driveChassisSpeeds, // Method that will drive the robot given ChassisSpeeds
        7,1.3,
        new ReplanningConfig(false, false), // Default path replanning config. See the API for the options here
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

  public double getLeftPosition () {
    return m_leftEncoder.getPosition();
  }

public double getRightPosition () {
    return m_rightEncoder.getPosition();
  }

  public double getLeftVelocity () {
    return  m_leftEncoder.getVelocity();
  }
  
  public double getRightVelocity () {
    return  m_rightEncoder.getVelocity();
  }

  public void resetEncoder () {
    REVLibError errorLeft = m_leftEncoder.setPosition(0);
    REVLibError errorRight = m_rightEncoder.setPosition(0);
    DashboardHelper.putString(DashboardHelper.LogLevel.Debug, "Left encoder errror", errorLeft.toString());
    DashboardHelper.putString(DashboardHelper.LogLevel.Debug, "Right encoder errror", errorRight.toString());    
  }
  


  DifferentialDriveOdometry m_odometry;

  public void periodic() {
    pose = m_odometry.update(getRotation(),
    getLeftPosition(),
    getRightPosition());

    field2d.setRobotPose(pose);

    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Left Encoder Position", getLeftPosition());
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Right Encoder Position", getRightPosition());
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Left Encoder Velocity", getLeftVelocity());
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Right Encoder Velocity", getRightVelocity());
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "gyro", getRotation().getDegrees());

    gyroEntry.append(getRotation().getDegrees());
    leftEncoderPositionEntry.append(getLeftPosition());
    rightEncoderPositionEntry.append(getRightPosition());
    leftEncoderVelocityEntry.append(getLeftVelocity());
    rightEncoderVelocityEntry.append(getRightVelocity());

  }

  public void resetPose(Pose2d newPose) {
    gyro.reset();
    resetEncoder();

    m_odometry = new DifferentialDriveOdometry(
          new Rotation2d(),
          0, 0,
          newPose);
    pose = m_odometry.getPoseMeters();



    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Rotation via pose at reset", pose.getRotation().getDegrees());
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Left Encoder Position at reset",getLeftPosition());
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Right Encoder Position at reset", getRightPosition());
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Pose x at reset", pose.getX());
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Pose y at reset", pose.getY());
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Gyro rotation at reset", getRotation().getDegrees());

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
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    return kinematics.toChassisSpeeds(wheelSpeeds);
  }
  
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "left m/s", speeds.leftMetersPerSecond);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "right m/s", speeds.rightMetersPerSecond);
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
    // final double leftFeedforward = 0;
    // final double rightFeedforward = 0;
    final double leftOutput =
        leftController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightController.calculate(getRightVelocity(), speeds.rightMetersPerSecond);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "right pid output", rightOutput);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Important, "left pid output", leftOutput);
    
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
    //DashboardHelper.putData(1, "Simulation Field", field2d);
    SmartDashboard.putData(field2d);

    DifferentialDrivetrainSim simulation = drivetrain.getSimulation();
    simulationGyro = simulation.getHeading();

    RelativeEncoderSim leftEncoder = (RelativeEncoderSim) m_leftEncoder;
    RelativeEncoderSim rightEncoder = (RelativeEncoderSim) m_rightEncoder;
    leftEncoder.setSimulationPosition(simulation.getLeftPositionMeters());
    rightEncoder.setSimulationPosition(simulation.getRightPositionMeters());
    leftEncoder.setSimulationVelocity(simulation.getLeftVelocityMetersPerSecond());
    rightEncoder.setSimulationVelocity(simulation.getRightVelocityMetersPerSecond());

  }
}
