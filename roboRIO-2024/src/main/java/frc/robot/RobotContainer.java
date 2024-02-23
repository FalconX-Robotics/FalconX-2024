// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.PIDShoot;
import frc.robot.commands.PathfindToPose;
import frc.robot.commands.RunIndex;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SmartDashboardPIDShoot;
import frc.robot.commands.SimpleShoot;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurboMode;
import frc.robot.commands.TurnInPlace;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.Sensor;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  private final XboxController driveController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController noteController = new XboxController(OperatorConstants.kShooterControllerPort);

  private final Settings m_settings = new Settings(driveController, noteController);

  private final Drivetrain m_drivetrain = new Drivetrain(m_settings);
  private final TankDrive m_tankDrive = new TankDrive(m_drivetrain, driveController);
  private final ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drivetrain, m_settings);
  private final CurvatureDrive m_curvatureDrive = new CurvatureDrive(m_drivetrain, m_settings);

  private final LEDs m_leds = new LEDs();
  private final Sensor m_sensor = new Sensor();

  private final Shooter m_shooter = new Shooter(m_settings);
  private final Intake m_intake = new Intake();
  private final Index m_index = new Index();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();


    SmartDashboard.putData("Auto Chooser", autoChooser);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
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
    Trigger turboModeTrigger = new JoystickButton(driveController, m_settings.driveController.getTurboButton().value);
    turboModeTrigger.whileTrue(new TurboMode(m_drivetrain));

    Trigger turnInPlaceTrigger = new JoystickButton(driveController, m_settings.driveController.getTurnInPlaceButton().value);
    turnInPlaceTrigger.whileTrue(new TurnInPlace(m_drivetrain));

    Trigger shooterTrigger = new JoystickButton(noteController, m_settings.noteController.getShooterButton().value);
    shooterTrigger.whileTrue(new PIDShoot(m_index, m_shooter));

    Trigger indexTrigger = new JoystickButton(noteController, m_settings.noteController.getIndexButton().value);
    indexTrigger.whileTrue(new RunIndex(m_index, .5).until(() -> {return !m_sensor.getNoteSensed();}));

    Trigger intakeTrigger = new JoystickButton(noteController, m_settings.noteController.getIntakeButton().value);
    intakeTrigger.whileTrue(new RunIntake(m_intake, 1.).until(() -> {return !m_sensor.getNoteSensed();}));

    Trigger temporaryTrigger = new JoystickButton(noteController, XboxController.Button.kY.value);
    temporaryTrigger.whileTrue(new RunIndex(m_index, 1.));

    Trigger reverseTrigger = new JoystickButton(noteController, m_settings.noteController.getReverseButton().value);
    reverseTrigger.whileTrue(new RunIndex(m_index, -.5)).whileTrue(new RunIntake(m_intake, -1.));

    m_drivetrain.setDefaultCommand(m_curvatureDrive);
    // m_shooter.setDefaultCommand(new SmartDashboardShoot(m_shooter, m_intake));//TODO: delete later :)
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
