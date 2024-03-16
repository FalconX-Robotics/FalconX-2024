// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmFeedForwardConstants;
import java.time.LocalDateTime;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.DashboardHelper.LogLevel;
import frc.robot.commands.ArmGoToGoalRotation;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.PIDShoot;
import frc.robot.commands.PathfindToPose;
import frc.robot.commands.ResetArmEncoder;
import frc.robot.commands.RunIndex;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SimpleShoot;
import frc.robot.commands.TriggerClimb;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Sensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LEDs.Color;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.DashboardHelper;
import frc.robot.subsystems.Sensor;

import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.concurrent.TimeUnit;

import org.ejml.data.ComplexPolar_F32;
import org.ejml.ops.ComplexMath_F64;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  
  private final SendableChooser<LogLevel> logLevelChooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser;
  
  private final XboxController driveController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController noteController = new XboxController(OperatorConstants.kShooterControllerPort);

  private final Settings m_settings = new Settings(driveController, noteController);

  private final Drivetrain m_drivetrain = new Drivetrain(m_settings);
  private final Arm m_arm = new Arm(m_settings);
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Sensor m_sensor = new Sensor();
  private final Index m_index = new Index();
  // private final Vision m_vision = new Vision();

  private final TankDrive m_tankDrive = new TankDrive(m_drivetrain, driveController);
  private final ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drivetrain, m_settings);
  private final CurvatureDrive m_curvatureDrive = new CurvatureDrive(m_drivetrain, m_settings);

  public final LEDs m_leds = new LEDs();
  // private final Vision m_vision = new Vision();
  

  public void periodic() {
    // m_vision.getAngleToTarget();
    // DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "PV Angle", m_vision.getAngleToTarget().orElse(0.));
    // m_vision.getAngleToTarget();
    // SmartDashboard.putNumber("PV Angle", m_vision.getAngleToTarget().orElse(0.));
    DashboardHelper.putString(LogLevel.Debug, "Arm Command", m_arm.getCurrentCommand() == null?"No Command":"Command Running");
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    NamedCommands.registerCommand("Shoot", new ParallelCommandGroup(
      new PIDShoot(m_shooter),
      //arm go to 5 degrees
      new ArmGoToGoalRotation(m_arm, Math.toRadians(5.)),
      //run index when shooter is at velocity and arm is at angle
      new RunIndex(m_index, 0.)
        .until(() -> {
          return m_shooter.velocityIsWithinTarget(2450., 50.)
          && Math.abs(Math.toDegrees(m_arm.getRotation()) - 5) <= 0.5;}
        )
        .withTimeout(.4)
        .andThen(new RunIndex(m_index, 1.)),
      new RunIntake(m_intake, -.5)
    )
    .withTimeout(0.8)
    .andThen(
      new ArmGoToGoalRotation(m_arm, 0)
      .withTimeout(.1)
    ));
    
    NamedCommands.registerCommand("Shoot Corner", new ParallelCommandGroup(
      new PIDShoot(m_shooter),
      new ArmGoToGoalRotation(m_arm, Math.toRadians(8.)),
      new RunIndex(m_index, 0.)
      .until(() -> {
        return m_shooter.velocityIsWithinTarget(2450., 50.) 
        && Math.abs(Math.toDegrees(m_arm.getRotation()) - 8) <= 0.5;}
      )
      .withTimeout(.4)
      .andThen(new RunIndex(m_index, 1.)),
      new RunIntake(m_intake, -.5)
    )
    .withTimeout(.8)
    .andThen(
      new ArmGoToGoalRotation(m_arm, 0)
      .withTimeout(.1)
    ));

    NamedCommands.registerCommand("Intake", 
      new RunIntake(m_intake, -0.6)
        .alongWith(new ArmGoToGoalRotation(m_arm, 0).withTimeout(.1))
        .alongWith(new RunIndex(m_index, 0.75))
        .until(() -> {return m_sensor.getNoteSensed();})
        .andThen(
          new RunIndex(m_index, -.3)
          .withTimeout(.15)
        ).alongWith(
          new SimpleShoot(m_shooter, -.2)
          .withTimeout(.15)
        )
      );
    
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Pathfind VERY EXPIREMENTAL", PathfindToPose.getPathfindCommand(1.23, 1.65, -127));
    autoChooser.addOption("Test every motor", new SequentialCommandGroup(
      Commands.run(()->{m_drivetrain.setLeftMotors(0.3);}, m_drivetrain).withTimeout(1),
      Commands.run(()->{m_drivetrain.setRightMotors(0.3);}, m_drivetrain).withTimeout(1),
      new ArmGoToGoalRotation(m_arm, Math.toRadians(95)),
      new ArmGoToGoalRotation(m_arm, Math.toRadians(5.)),
      new RunIntake(m_intake, 0.3).withTimeout(1),
      new RunIndex(m_index, 0.3).withTimeout(1),
      new SimpleShoot(m_shooter, 0.3)
    ));

    logLevelChooser.setDefaultOption("Info", DashboardHelper.LogLevel.Info);
    logLevelChooser.addOption("Important", DashboardHelper.LogLevel.Important);
    logLevelChooser.addOption("Debug", DashboardHelper.LogLevel.Debug);
    logLevelChooser.addOption("Verbose", DashboardHelper.LogLevel.Verbose);

    DashboardHelper.putData(DashboardHelper.LogLevel.Info, "LogLevel Choices", logLevelChooser);
    DashboardHelper.putData(DashboardHelper.LogLevel.Info, "Auto Chooser", autoChooser);
    

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
   * {@lnk Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_settings.noteSettings.intakeTrigger.whileTrue(
      new RunIntake(m_intake, -0.8)
      .alongWith(new RunIndex(m_index, 1.))
      .until(() -> {return m_sensor.getNoteSensed();})
      .andThen(
        new RunIndex(m_index, -.3)
        .withTimeout(.15)
      ).alongWith(
        new SimpleShoot(m_shooter, -.2)
        .withTimeout(.15)
      )
    );
    m_settings.noteSettings.shooterChargeTrigger.whileTrue(
      new PIDShoot(m_shooter)
      .alongWith(
        new ArmGoToGoalRotation(m_arm, Math.toRadians(5))
      )
    ).onFalse(
      new ArmGoToGoalRotation(m_arm, 0)
      .withTimeout(.3)
    );
    m_settings.noteSettings.shooterFireTrigger.whileTrue(
      new RunIndex(m_index, 1.)
      .onlyIf(() -> {return m_shooter.velocityIsWithinTarget(2650., 25.);})
      .withTimeout(1.5).andThen(new RunIndex(m_index, .5))
    );
    m_settings.noteSettings.reverseTrigger.whileTrue(
      new RunIndex(m_index, -.5)
      .alongWith(new RunIntake(m_intake, 1.))
    );
    m_settings.noteSettings.shootAmpTrigger.whileTrue(new RunIndex(m_index, .5).alongWith(new SimpleShoot(m_shooter, .6)));

    new Trigger(()->  {return m_sensor.getNoteSensed();}).whileTrue(
      Commands.run(()->{
        //m_vision.poseOffsetLedIndicator();
      Optional<Double> angle = m_vision.getAngleToTarget();
      if (angle.isEmpty()) {
        m_leds.setColor(LEDs.Color.FOREST);
      }, m_leds)
    );
    m_leds.setDefaultCommand(Commands.run(()->{
      var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
              m_leds.setColor(Color.RED);
            } else if (alliance.get() == DriverStation.Alliance.Blue){
              m_leds.setColor(Color.BLUE);
            } else {
              m_leds.setColor(Color.PURPLE);
            }
          } else {
            m_leds.setColor(Color.PURPLE);
          }        DashboardHelper.putString(LogLevel.Info, "Angle alignment to target", "Target Not Present.");
      } else if(Math.abs(angle.get()) < 5) {
        m_vision.ledColor.setColor(m_vision.ledsIsAligned);
        DashboardHelper.putString(LogLevel.Info, "Angle alignment to target", "Aligned.");
                
        } else if(Math.abs(angle.get()) > 5) {
        m_leds.setColor(LEDs.Color.LAVA);
        DashboardHelper.putString(LogLevel.Info, "Angle alignment to target", "Unaligned.");
      }

    }, m_leds, m_vision));

    m_settings.noteSettings.resetArmEncoderTrigger.onTrue(new ResetArmEncoder(m_arm));

    m_settings.noteSettings.ampTrigger.onTrue(new ArmGoToGoalRotation(m_arm, Math.toRadians(95)).onlyWhile(() -> {return !m_arm.armJoystickActive();}));
    m_settings.noteSettings.storeTrigger.onTrue(new ArmGoToGoalRotation(m_arm, Math.toRadians(5.)).onlyWhile(() -> {return !m_arm.armJoystickActive();})
      .withTimeout(3.));

    m_drivetrain.setDefaultCommand(m_curvatureDrive);
    m_climber.setDefaultCommand(new TriggerClimb(m_settings, m_climber));
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
