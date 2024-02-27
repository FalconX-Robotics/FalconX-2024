// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.PathfindToPose;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurboMode;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.OdometrySubsystem;

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
  // The robot's subsystems and commands are defined here...
  // final Settings m_settings = new Settings(driveController, noteController);

  public final Drivetrain m_drivetrain = new Drivetrain(m_settings);
  public final TankDrive m_tankDrive = new TankDrive(m_drivetrain, driveController);
  final ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drivetrain, m_settings);
  final CurvatureDrive m_curvatureDrive = new CurvatureDrive(m_drivetrain, m_settings);

  public final LEDs m_leds = new LEDs();
  
  final Shooter m_shooter = new Shooter();
  final Intake m_intake = new Intake();  
  final Vision m_vision = new Vision();

  public void periodic() {
    m_vision.getAngleToTarget();
    SmartDashboard.putNumber("PV Angle", m_vision.getAngleToTarget());
  }


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

    Trigger rightBumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
    rightBumper.whileTrue(new TurboMode(m_drivetrain));

    m_drivetrain.setDefaultCommand(m_curvatureDrive);
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
