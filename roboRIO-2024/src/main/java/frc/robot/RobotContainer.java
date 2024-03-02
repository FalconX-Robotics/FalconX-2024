// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.DashboardHelper.LogLevel;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmGoToGoalRotation;
import frc.robot.commands.BackupAuto;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.PIDShoot;
import frc.robot.commands.PathfindToPose;
import frc.robot.commands.RunIndex;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurboMode;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.DashboardHelper;
import frc.robot.subsystems.Sensor;

import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.concurrent.TimeUnit;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<LogLevel> logLevelChooser = new SendableChooser<>();

  private final XboxController driveController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController noteController = new XboxController(OperatorConstants.kShooterControllerPort);

  private final Settings m_settings = new Settings(driveController, noteController);

  private final Drivetrain m_drivetrain = new Drivetrain(m_settings);
  private final Arm m_arm = new Arm();
  private final Shooter m_shooter = new Shooter(m_settings);
  private final Intake m_intake = new Intake();
  private final Index m_index = new Index();

  private final TankDrive m_tankDrive = new TankDrive(m_drivetrain, driveController);
  private final ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drivetrain, m_settings);
  private final CurvatureDrive m_curvatureDrive = new CurvatureDrive(m_drivetrain, m_settings);

  public final LEDs m_leds = new LEDs();

 
  private final Vision m_vision = new Vision();
  private final Sensor m_sensor = new Sensor();

  public void periodic() {
    m_vision.getAngleToTarget();
    SmartDashboard.putNumber("PV Angle", m_vision.getAngleToTarget().orElse(0.));
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("backupAuto", new BackupAuto(m_drivetrain));

    logLevelChooser.setDefaultOption("Info", DashboardHelper.LogLevel.Info);
    logLevelChooser.addOption("Important", DashboardHelper.LogLevel.Important);
    logLevelChooser.addOption("Debug", DashboardHelper.LogLevel.Debug);
    logLevelChooser.addOption("Verbose", DashboardHelper.LogLevel.Verbose);

    DashboardHelper.putData(DashboardHelper.LogLevel.Info, "Auto Chooser", autoChooser);
    DashboardHelper.putData(DashboardHelper.LogLevel.Info, "LogLevel Choices", logLevelChooser);

    LocalDateTime startTime = LocalDateTime.now();
    Util.setStartTime(startTime);
    DataLogManager.start();
    
    DriverStation.startDataLog(DataLogManager.getLog());
    logLevelChooser.onChange((logLevel) -> {
      DashboardHelper.setLogLevel(logLevel);
    });
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_settings.driveController.turboModeTrigger.whileTrue(
      new TurboMode(m_drivetrain)
    );

    m_settings.noteController.shooterChargeTrigger.whileTrue(
      new PIDShoot(m_index, m_shooter)
    );
    m_settings.noteController.shooterFireTrigger.whileTrue(
      new RunIndex(m_index, 1.)
      .onlyIf(() -> {return m_shooter.velocityIsWithinTarget();})
    );
    m_settings.noteController.intakeTrigger.whileTrue(
      new RunIntake(m_intake, -0.8)
      .until(() -> {return m_sensor.getNoteSensed();})
    );
    m_settings.noteController.reverseTrigger.whileTrue(
      new RunIndex(m_index, -.5)
      .alongWith(new RunIntake(m_intake, 1.))
    );

    m_drivetrain.setDefaultCommand(m_curvatureDrive);
    m_arm.setDefaultCommand(new ArmGoToGoalRotation(m_arm, 0.));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public LogLevel getSelectedLogLevel(){
    return logLevelChooser.getSelected();
  }
  
}
