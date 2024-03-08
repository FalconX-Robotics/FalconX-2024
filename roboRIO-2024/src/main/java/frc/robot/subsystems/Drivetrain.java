// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RatioConstants;
import frc.robot.DashboardHelper.LogLevel;
import frc.robot.DashboardHelper;

public class Drivetrain extends SubsystemBase {
  // There is a different system used than previous years because MotorControlGroup is deprecated :(.
  // We set a motor to a leader, and make followers follow the leader in the constructor.  
  // Front wheels are leaders for no reason because its redundant
  private CANSparkMax leftLeader = new CANSparkMax(MotorConstants.DrivetrainMotors.frontLeft.value, MotorType.kBrushless);
  private CANSparkMax rightLeader = new CANSparkMax(MotorConstants.DrivetrainMotors.frontRight.value, MotorType.kBrushless);
  private CANSparkMax leftFollower = new CANSparkMax(MotorConstants.DrivetrainMotors.backLeft.value, MotorType.kBrushless);
  private CANSparkMax rightFollower = new CANSparkMax(MotorConstants.DrivetrainMotors.backRight.value, MotorType.kBrushless);

  private DataLog log = DataLogManager.getLog();
  
  DoubleLogEntry leftVoltageEntry = new DoubleLogEntry(log, "/dt/leftVoltage");
  DoubleLogEntry rightVoltageEntry = new DoubleLogEntry(log, "/dt/rightVoltage");
  

  private WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.PIGEON_PORT);

  private OdometrySubsystem m_odometry;

  private Settings m_settings;

  public void setLeftMotors (double volt) {
    leftLeader.set(volt);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Left Motors Set", volt);
    
    leftVoltageEntry.append(leftLeader.get() * RobotController.getBatteryVoltage());
    
  }
  public void setRightMotors (double volt) {
    rightLeader.set(volt);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Right Motors Set", volt);

    rightVoltageEntry.append(rightLeader.get() * RobotController.getBatteryVoltage());
  }

  /** Creates a new Drivetrain. */
  public Drivetrain(Settings settings) {
    m_settings = settings;

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setInverted(false);
    leftFollower.setInverted(false);
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);
    leftLeader.getEncoder().setMeasurementPeriod(8);
    rightLeader.getEncoder().setMeasurementPeriod(8);
    leftLeader.getEncoder().setAverageDepth(2);
    rightLeader.getEncoder().setAverageDepth(2);
    leftLeader.setSmartCurrentLimit(60);
    rightLeader.setSmartCurrentLimit(60);
    leftLeader.setSmartCurrentLimit(60);
    rightLeader.setSmartCurrentLimit(60);
    applytoAllMotors((motor) -> {motor.setOpenLoopRampRate(0.2);});

    applytoAllMotors((motor) -> {
      motor.setIdleMode(IdleMode.kBrake);
    });

    setMotorConversionFactors();

    m_odometry = new OdometrySubsystem(this);
  }

  private void applytoAllMotors(Consumer<CANSparkMax> consumer) {
    consumer.accept(leftLeader);
    consumer.accept(leftFollower);
    consumer.accept(rightLeader);
    consumer.accept(rightFollower);
  }

  private void setMotorConversionFactors() {
    
    double conversionFactor = 1./(RatioConstants.KITBOT_GEAR_RATIO) * BaseUnits.Distance.convertFrom(Constants.KITBOT_WHEEL_DIAMETER * Math.PI, Units.Inches);
    applytoAllMotors((motor) -> {
      motor.getEncoder().setVelocityConversionFactor(conversionFactor/60);
    });
    applytoAllMotors((motor) -> {
      motor.getEncoder().setPositionConversionFactor(conversionFactor);
    });
    
    // m/s
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
    m_drive.feed();
  }

  public void setLeftMotorsVoltage(double voltage) {
    leftLeader.setVoltage(voltage);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Left Motor Voltage", voltage);
    leftVoltageEntry.append(voltage);
  }

  public void setRightMotorsVoltage(double voltage) {
    rightLeader.setVoltage(voltage);
    DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "Right Motor Voltage", voltage);
    rightVoltageEntry.append(voltage);
  }

  public DifferentialDrive m_drive = new DifferentialDrive(leftLeader, rightLeader);

  public void arcadeDrive (double speed, double rotation, boolean turbo){
    m_drive.arcadeDrive(
      speed * (turbo ? m_settings.driveSettings.turboSpeed : m_settings.driveSettings.normalSpeed),
      rotation * (turbo ? m_settings.driveSettings.turboSpeed : m_settings.driveSettings.normalSpeed)
    );
  }

  public void curvatureDrive (double speed, double rotation, boolean turbo, boolean turnInPlace){
    
    speed = speed * (turbo ? m_settings.driveSettings.turboSpeed : m_settings.driveSettings.normalSpeed);
    if (!turbo && turnInPlace) {
      rotation *= m_settings.driveSettings.normalSpeed;
    } else if (turbo && !turnInPlace) {
      rotation *= 1./2.;
    }

    DashboardHelper.putNumber(LogLevel.Debug, "Rotation", rotation);
    WheelSpeeds wheelSpeeds = DifferentialDrive.curvatureDriveIK(speed, rotation, turnInPlace);
    wheelSpeeds.left += 0.00 * Math.signum(wheelSpeeds.left);
    wheelSpeeds.right += 0.00 * Math.signum(wheelSpeeds.right);

    setLeftMotors(wheelSpeeds.left);
    setRightMotors(wheelSpeeds.right);
    m_drive.feed();
  }
  @Override
  public void periodic() {
    m_odometry.periodic();
    DashboardHelper.putBoolean(DashboardHelper.LogLevel.Info, "turnInPlace", m_settings.driveSettings.turnInPlaceTrigger.getAsBoolean());
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
  new DifferentialDrivetrainSim(DCMotor.getNEO(2), Constants.RatioConstants.KITBOT_GEAR_RATIO, 5, 
  BaseUnits.Mass.convertFrom(95, Units.Pounds), BaseUnits.Distance.convertFrom(2, Units.Inches), 
  Units.Inches.toBaseUnits(22), null);
  

  @Override
  public void simulationPeriodic() {
    if (!DriverStation.isDisabled()) {
      m_odometry.simulationPeriodic();
      m_simulation.setInputs(leftLeader.get() * RobotController.getBatteryVoltage(), rightLeader.get() * RobotController.getBatteryVoltage());
      m_simulation.update(0.02);
    }
  }

  DifferentialDrivetrainSim getSimulation() {
    return m_simulation;
  }

  
}
