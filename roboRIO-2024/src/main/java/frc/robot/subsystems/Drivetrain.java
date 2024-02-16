// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
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
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  private DataLog log = DataLogManager.getLog();
  
  DoubleLogEntry leftVoltageEntry = new DoubleLogEntry(log, "/dt/leftVoltage");
  DoubleLogEntry rightVoltageEntry = new DoubleLogEntry(log, "/dt/rightVoltage");
  

  private WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.PIGEON_PORT);

  private OdometrySubsystem m_odometry;
  private State state;


  

  public boolean turboModeOn = false;

  public void setLeftMotors (double volt) {
    leftLeader.set(volt);
    SmartDashboard.putNumber("Left Motors Set", volt);
    
    leftVoltageEntry.append(leftLeader.get() * RobotController.getBatteryVoltage());
    
  }
  public void setRightMotors (double volt) {
    rightLeader.set(volt);
    SmartDashboard.putNumber("Right Motors Set", volt);
    rightVoltageEntry.append(rightLeader.get() * RobotController.getBatteryVoltage());
  }

  public void logMotors(SysIdRoutineLog log) {
    log.recordState(state);
  }

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setInverted(false);
    leftFollower.setInverted(false);
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);
    leftLeader.getEncoder().setMeasurementPeriod(20);
    rightLeader.getEncoder().setMeasurementPeriod(20);
    leftLeader.getEncoder().setAverageDepth(1);
    rightLeader.getEncoder().setAverageDepth(1);

    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);

    setMotorConversionFactors();

    m_odometry = new OdometrySubsystem(this);
  }
  private void setMotorConversionFactors() {
    double conversionFactor = 1./(Constants.NESSIE_GEAR_RATIO) * BaseUnits.Distance.convertFrom(6 * Math.PI, Units.Inches);
    leftLeader.getEncoder().setVelocityConversionFactor(conversionFactor/60);
    leftFollower.getEncoder().setVelocityConversionFactor(conversionFactor/60);
    rightLeader.getEncoder().setVelocityConversionFactor(conversionFactor/60);
    rightFollower.getEncoder().setVelocityConversionFactor(conversionFactor/60);
    
    leftLeader.getEncoder().setPositionConversionFactor(conversionFactor);
    leftFollower.getEncoder().setPositionConversionFactor(conversionFactor);
    rightLeader.getEncoder().setPositionConversionFactor(conversionFactor);
    rightFollower.getEncoder().setPositionConversionFactor(conversionFactor); // m/s
  }

  // runs the motors
  public void setMotors(double left, double right) {
    setLeftMotors(left);
    setRightMotors(right);
  }

  public void setMotorVoltage(double leftVoltage, double rightVoltage) {

    if(Robot.isSimulation()) {
      setLeftMotors(leftVoltage/12);
      setRightMotors(rightVoltage/12);
    } else {
      setLeftMotorsVoltage(leftVoltage);
      setRightMotorsVoltage(rightVoltage);
    }
  }

  public void setLeftMotorsVoltage(double voltage) {
    leftLeader.setVoltage(voltage);
    SmartDashboard.putNumber("Left Motor Voltage", voltage);
    leftVoltageEntry.append(voltage);
  }

  public void setRightMotorsVoltage(double voltage) {
    rightLeader.setVoltage(voltage);
    SmartDashboard.putNumber("Right Motor Voltage", voltage);
    rightVoltageEntry.append(voltage);
  }

  public DifferentialDrive m_drive = new DifferentialDrive(leftLeader, rightLeader);

  public void arcadeDrive (double speed, double rotation){
    m_drive.arcadeDrive(speed * (turboModeOn ? 1 : (1./2.)), rotation * (turboModeOn ? 1 : (1./2.)));
  }

  public void curvatureDrive (double speed, double rotation, boolean allowTurnInPlace){
    m_drive.curvatureDrive(speed * (turboModeOn ? 1 : (1./2.)), rotation, allowTurnInPlace);
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
    m_simulation.setInputs(leftLeader.get() * RobotController.getBatteryVoltage(), rightLeader.get() * RobotController.getBatteryVoltage());
    m_simulation.update(0.02);
  }

  DifferentialDrivetrainSim getSimulation() {
    return m_simulation;
  }

  
}
