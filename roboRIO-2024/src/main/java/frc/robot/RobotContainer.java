// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MotorConstants;
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
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.Sensors;

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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driveController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController noteController = new XboxController(OperatorConstants.kShooterControllerPort);

  private final Settings m_settings = new Settings(driveController, noteController);

  final Drivetrain m_drivetrain = new Drivetrain(m_settings);
  final TankDrive m_tankDrive = new TankDrive(m_drivetrain, driveController);
  final ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drivetrain, m_settings);
  final CurvatureDrive m_curvatureDrive = new CurvatureDrive(m_drivetrain, m_settings);

  final LEDs m_leds = new LEDs();
  
  final Shooter m_shooter = new Shooter(m_settings);
  final Intake m_intake = new Intake();
  final Index m_index = new Index();
  final Sensors m_sensor = new Sensors();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // MotorConstants.initializeConstants();
    
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

    Trigger turboModeTrigger = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
    turboModeTrigger.whileTrue(new TurboMode(m_drivetrain));

    Trigger intakeTrigger = new JoystickButton(noteController, XboxController.Button.kA.value);
    intakeTrigger.whileTrue(new RunIntake(m_intake, 1.)
    // .until(() -> {return m_sensor.getNoteSensed();})
    );
    
    Trigger UNintakeTrigger = new JoystickButton(noteController, XboxController.Button.kRightBumper.value);
    UNintakeTrigger.whileTrue(new RunIntake(m_intake, -.7));

    Trigger indexTrigger = new JoystickButton(noteController, XboxController.Button.kA.value);
    indexTrigger.whileTrue(new RunIndex(m_index, 1.));

    Trigger UNindexTrigger = new JoystickButton(noteController, XboxController.Button.kLeftBumper.value);
    UNindexTrigger.whileTrue(new RunIndex(m_index, -.5));

    Trigger simpleShootTrigger = new JoystickButton(noteController, XboxController.Button.kX.value);
    simpleShootTrigger.whileTrue(new PIDShoot(m_shooter));

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
