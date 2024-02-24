// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import java.util.function.Consumer;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.MotorConstants.drivetrain;

public class Drivetrain extends SubsystemBase {
  // There is a different system used than previous years because MotorControlGroup is deprecated :(.
  // We set a motor to a leader, and make followers follow the leader in the constructor.  
  // Front wheels are leaders for no reason because its redundant
  private CANSparkMax leftLeader = new CANSparkMax(MotorConstants.drivetrain.frontLeft.value, MotorType.kBrushless);
  private CANSparkMax rightLeader = new CANSparkMax(MotorConstants.drivetrain.frontRight.value, MotorType.kBrushless);
  private CANSparkMax leftFollower = new CANSparkMax(MotorConstants.drivetrain.backLeft.value, MotorType.kBrushless);
  private CANSparkMax rightFollower = new CANSparkMax(MotorConstants.drivetrain.backRight.value, MotorType.kBrushless);

  private DataLog log = DataLogManager.getLog();
  
  DoubleLogEntry leftVoltageEntry = new DoubleLogEntry(log, "/dt/leftVoltage");
  DoubleLogEntry rightVoltageEntry = new DoubleLogEntry(log, "/dt/rightVoltage");
  

  private WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.PIGEON_PORT);

  private OdometrySubsystem m_odometry;
  private State state;

  private Settings m_settings;

  public void setLeftMotors (double volt) {
    leftLeader.set(volt);
    SmartDashboard.putNumber("Left Motors Set", volt);
    
    leftVoltageEntry.append(leftLeader.get() * RobotController.getBatteryVoltage());
    
  }
  public void setRightMotors (double volt) {
    rightLeader.set(volt);
  }

  /** Creates a new Drivetrain. */
  public Drivetrain(Settings settings) {
    m_settings = settings;

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    applyToAllMotors((motor) -> {
      motor.setSmartCurrentLimit(Constants.DRIVETRAIN_CURRENT_LIMIT);
      motor.setIdleMode(IdleMode.kBrake);
      motor.setOpenLoopRampRate(Constants.DRIVETRAIN_RAMP_RATE);
    });

    leftLeader.setInverted(false);
    leftFollower.setInverted(false);
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);
    leftLeader.getEncoder().setMeasurementPeriod(8);
    rightLeader.getEncoder().setMeasurementPeriod(8);
    leftLeader.getEncoder().setAverageDepth(4);
    rightLeader.getEncoder().setAverageDepth(4);

    setMotorConversionFactors();

    m_odometry = new OdometrySubsystem(this);
  }

  private void applyToAllMotors(Consumer<CANSparkMax> motorConsumer) {
    motorConsumer.accept(leftLeader);
    motorConsumer.accept(leftFollower);
    motorConsumer.accept(rightLeader);
    motorConsumer.accept(rightFollower);
  }

  private void setMotorConversionFactors() {
    double conversionFactor = 1./(RatioConstants.NESSIE_GEAR_RATIO) * BaseUnits.Distance.convertFrom(6 * Math.PI, Units.Inches);
    applyToAllMotors((motor) -> {
      motor.getEncoder().setVelocityConversionFactor(conversionFactor/60); //meters per second
      motor.getEncoder().setPositionConversionFactor(conversionFactor); //meters
    });

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

  public void arcadeDrive (double speed, double rotation, boolean turboModeOn){
    m_drive.arcadeDrive(
      speed * (turboModeOn ? m_settings.driveController.turboSpeed : m_settings.driveController.normalSpeed),
      rotation * (turboModeOn ? m_settings.driveController.turboSpeed : m_settings.driveController.normalSpeed)
    );
  }

  public void curvatureDrive (double speed, double rotation, boolean turnInPlace, boolean turboModeOn){
    WheelSpeeds wheelSpeeds = DifferentialDrive.curvatureDriveIK(speed * (turboModeOn ? m_settings.driveController.turboSpeed : m_settings.driveController.normalSpeed),
    rotation, turnInPlace);
    if (Math.abs(wheelSpeeds.left) < 0.0001) wheelSpeeds.left = 0;
    if (Math.abs(wheelSpeeds.right) < 0.0001) wheelSpeeds.right = 0;
    wheelSpeeds.left += Constants.LEFT_FRICTION_OFFSET * Math.signum(wheelSpeeds.left);
    wheelSpeeds.right += Constants.RIGHT_FRICTION_OFFSET * Math.signum(wheelSpeeds.right);

    setLeftMotors(wheelSpeeds.left);
    setRightMotors(wheelSpeeds.right);

    // m_drive.curvatureDrive(speed * (turboModeOn ? m_settings.driveController.turboSpeed : m_settings.driveController.normalSpeed), rotation, turnInPlace);
  }
  @Override
  public void periodic() {
    m_odometry.periodic();
    // SmartDashboard.putBoolean("turnInPlace", turnInPlace);
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

  // public void setTurnInPlace (boolean newTurnInPlace) {
  //   // turnInPlace = newTurnInPlace;
  // }
  // public boolean getTurnInPlace () {
  //   // return turnInPlace;
  // }
  // public void setTurboMode (boolean newTurboMode) {
  //   // turboModeOn = newTurboMode;
  // }
  // public boolean getTurboMode () {
  //   // return turboModeOn;
  // }

  DifferentialDrivetrainSim m_simulation = 
  new DifferentialDrivetrainSim(DCMotor.getNEO(2), Constants.RatioConstants.KITBOT_GEAR_RATIO, 5, 
  BaseUnits.Mass.convertFrom(95, Units.Pounds), BaseUnits.Distance.convertFrom(2, Units.Inches), 
  BaseUnits.Distance.convertFrom(23, Units.Inches), null);
  

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
