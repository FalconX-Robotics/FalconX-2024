// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.MotorConstants;
import frc.robot.simulation.RelativeEncoderSim;

public class Drivetrain extends SubsystemBase {
  // There is a different system used than previous years because MotorControlGroup is deprecated :(.
  // We set a motor to a leader, and make followers follow the leader in the constructor.  
  // Front wheels are leaders for no reason because its redundant
  private CANSparkMax leftLeader = new CANSparkMax(MotorConstants.frontLeft, MotorType.kBrushless);
  private CANSparkMax rightLeader = new CANSparkMax(MotorConstants.frontRight, MotorType.kBrushless);
  private CANSparkMax leftFollower = new CANSparkMax(MotorConstants.backLeft, MotorType.kBrushless);
  private CANSparkMax rightFollower = new CANSparkMax(MotorConstants.backRight, MotorType.kBrushless);

  private WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.PIGEON_PORT);

  private OdometrySubsystem m_odometry;

  public boolean turboModeOn = false;

  public void setLeftMotors (double volt) {
    leftLeader.set(volt);
  }
  public void setRightMotors (double volt) {
    rightLeader.set(volt);
  }

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    rightLeader.setInverted(false);
    rightFollower.setInverted(false);

    setMotorConversionFactors();

    m_odometry = new OdometrySubsystem(this);
  }
  private void setMotorConversionFactors() {
    leftLeader.getEncoder().setVelocityConversionFactor(4. * Constants.NESSIE_GEAR_RATIO);
    leftFollower.getEncoder().setVelocityConversionFactor(4. * Constants.NESSIE_GEAR_RATIO);
    rightLeader.getEncoder().setVelocityConversionFactor(4. * Constants.NESSIE_GEAR_RATIO);
    rightFollower.getEncoder().setVelocityConversionFactor(4. * Constants.NESSIE_GEAR_RATIO);
    
    leftLeader.getEncoder().setPositionConversionFactor(4. * Constants.NESSIE_GEAR_RATIO);
    leftFollower.getEncoder().setPositionConversionFactor(4. * Constants.NESSIE_GEAR_RATIO);
    rightLeader.getEncoder().setPositionConversionFactor(4. * Constants.NESSIE_GEAR_RATIO);
    rightFollower.getEncoder().setPositionConversionFactor(4. * Constants.NESSIE_GEAR_RATIO); // m/s
  }

  // runs the motors
  public void setMotors(double left, double right) {
    setLeftMotors(left * (turboModeOn ? 1 : (1/3)));
    setRightMotors(right * (turboModeOn ? 1 : (1/3)));
  }

  public void setMotorVoltage(double leftVoltage, double rightVoltage) {
    System.out.println(leftVoltage + ", " + rightVoltage);

    if(Robot.isSimulation()) {
      setLeftMotors(leftVoltage/12);
      setRightMotors(rightVoltage/12);
    } else {
      setLeftMotorsVoltage(leftVoltage * (turboModeOn ? 1 : (1/3)));
      setRightMotorsVoltage(rightVoltage * (turboModeOn ? 1 : (1/3)));
    }
  }

  public void setLeftMotorsVoltage(double voltage) {
    leftLeader.setVoltage(voltage);
  }

  public void setRightMotorsVoltage(double voltage) {
    rightLeader.setVoltage(voltage);
  }

  public DifferentialDrive m_drive = new DifferentialDrive(leftLeader, rightLeader);

  public void arcadeDrive (double speed, double rotation){
    m_drive.arcadeDrive(speed * (turboModeOn ? 1 : (1/3)), rotation * (turboModeOn ? 1 : (1/2)));
  }

  public void curvatureDrive (double speed, double rotation, boolean allowTurnInPlace){
    m_drive.curvatureDrive(speed * (turboModeOn ? 1 : (1/3)), rotation * (turboModeOn ? 1 : (1/3)), allowTurnInPlace);
  }
  @Override
  public void periodic() {
    m_odometry.periodic();
  }

  public WPI_PigeonIMU getGyro() {
    return gyro;
  }

  public RelativeEncoder getRightEncoder() {
    return rightLeader.getEncoder();
  }


  public RelativeEncoder getLeftEncoder() {
    return leftLeader.getEncoder();
  }


  DifferentialDrivetrainSim m_simulation = 
  new DifferentialDrivetrainSim(DCMotor.getNEO(2), Constants.NESSIE_GEAR_RATIO, 5, 
  BaseUnits.Mass.convertFrom(120, Units.Pounds), BaseUnits.Distance.convertFrom(3, Units.Inches), 
  BaseUnits.Distance.convertFrom(18, Units.Inches), null);
  

  @Override
  public void simulationPeriodic() {
    m_odometry.simulationPeriodic();
    m_simulation.setInputs(leftLeader.get() * RobotController.getInputVoltage(), rightLeader.get() * RobotController.getInputVoltage());
    m_simulation.update(0.02);
  }

  DifferentialDrivetrainSim getSimulation() {
    return m_simulation;
  }
}
