// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmFeedForwardConstants;
import frc.robot.Constants.DIOConstants;

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
import frc.robot.commands.ArmStayInPlace;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbIndividual;
import frc.robot.commands.AutoRotate;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.PIDShoot;
import frc.robot.commands.PathfindToPose;
import frc.robot.commands.ResetArmEncoder;
import frc.robot.commands.RunIndex;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SimpleShoot;
import frc.robot.commands.TankDrive;
import frc.robot.commands.AimArm;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.TriggerClimb;
import frc.robot.commands.ClimbIndividual.Side;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LimitSwitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LEDs.Color;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.Sensor;
import frc.robot.DashboardHelper;
import frc.robot.subsystems.LimitSwitch;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

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

public final LEDs m_leds = new LEDs();
  private final Settings m_settings = new Settings(driveController, noteController);

  private final Drivetrain m_drivetrain = new Drivetrain(m_settings);
  private final Arm m_arm = new Arm(m_settings);
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Sensor m_sensor = new Sensor();
  private final LimitSwitch m_leftClimbLimitSwitch = new LimitSwitch(DIOConstants.LEFT_CLIMB_SWITCH);
  private final LimitSwitch m_rightClimbLimitSwitch = new LimitSwitch(DIOConstants.RIGHT_CLIMB_SWITCH);
  private final Index m_index = new Index();
  private final Climber m_climber = new Climber(m_leftClimbLimitSwitch, m_rightClimbLimitSwitch);
  private final Vision m_vision = new Vision(m_leds);

  private final TankDrive m_tankDrive = new TankDrive(m_drivetrain, driveController);
  private final ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drivetrain, m_settings);
  private final CurvatureDrive m_curvatureDrive = new CurvatureDrive(m_drivetrain, m_settings);

    

  public void periodic() {
    // m_vision.getAngleToTarget();
    // DashboardHelper.putNumber(DashboardHelper.LogLevel.Info, "PV Angle", m_vision.getAngleToTarget().orElse(0.));
    // m_vision.getAngleToTarget();
    // SmartDashboard.putNumber("PV Angle", m_vision.getAngleToTarget().orElse(0.));
    DashboardHelper.putString(LogLevel.Debug, "Arm Command", m_arm.getCurrentCommand() == null?"No Command":m_arm.getCurrentCommand().getName());
    DashboardHelper.putString(LogLevel.Debug, "Shooter Command", m_shooter.getCurrentCommand() == null?"No Command":m_shooter.getCurrentCommand().getName());
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Named commands are commands for use in PathPlanner -w
    NamedCommands.registerCommand("Shoot", new ParallelCommandGroup(
      new PIDShoot(m_shooter, m_index),
      //arm go to 5 degrees
      new ArmGoToGoalRotation(m_arm, Math.toRadians(ArmFeedForwardConstants.shootingAngleDegrees)),
      //run index when shooter is at velocity and arm is at angle
      new RunIndex(m_index, 0.)
        .until(() -> {
          return m_shooter.velocityIsWithinTarget(Constants.shooterSpeedAtSubwoofer, 50.)
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
      new PIDShoot(m_shooter, m_index),
      new ArmGoToGoalRotation(m_arm, Math.toRadians(8.)),
      new RunIndex(m_index, 0.)
      .until(() -> {
        return m_shooter.velocityIsWithinTarget(Constants.shooterSpeedAtSubwoofer, 50.) 
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
      new PIDShoot(m_shooter, m_index)
      .alongWith(
        new ArmGoToGoalRotation(m_arm, Math.toRadians(ArmFeedForwardConstants.shootingAngleDegrees))
      )
    ).onFalse(
      new ArmGoToGoalRotation(m_arm, 0)
      .withTimeout(.3)
    );
    m_settings.noteSettings.shooterFireTrigger.whileTrue(
      new RunIndex(m_index, 1.)
      .onlyIf(() -> {return m_shooter.velocityIsWithinTarget(1000., 25.);})
      .withTimeout(1.5).andThen(new RunIndex(m_index, .5))
    );
    m_settings.noteSettings.reverseTrigger.whileTrue(
      new RunIndex(m_index, -.5)
      .alongWith(new RunIntake(m_intake, 1.))
    );
    m_settings.noteSettings.shootAmpTrigger.whileTrue(new RunIndex(m_index, .5).alongWith(new SimpleShoot(m_shooter, .6)));
    m_settings.noteSettings.stayInPlaceTrigger.onTrue(new ArmStayInPlace(m_arm));

    m_settings.noteSettings.autoAimTrigger.whileTrue(new AimArm(m_arm, m_vision));
    m_settings.noteSettings.autoShootTrigger.whileTrue(new AutoShoot(m_shooter, m_vision, m_index, m_arm).withTimeout(1.5));
    m_settings.noteSettings.autoShootTrigger.onFalse(new ArmGoToGoalRotation(m_arm, Math.toRadians(0.5)).withTimeout(1.5));
    m_settings.driveSettings.autoRotateTrigger.whileTrue(new AutoRotate(m_drivetrain, m_vision));
    // m_settings.noteSettings.autoAimTrigger.whileTrue(
    //   Commands.run(()->{
    //     m_leds.setColor(Color.YELLOW);
    //   },
    //   m_arm, m_vision, m_leds)
    //   .until(m_settings.noteSettings::povIsActive)
    // );

    new Trigger(m_sensor::getNoteSensed).whileTrue(
      Commands.run(()->{
        //m_vision.poseOffsetLedIndicator()
      // Optional<Double> angle = m_vision.getAngleToTarget();
      // if (angle.isEmpty()) {
        m_leds.setColor(LEDs.Color.FOREST);
      // }
      }, m_leds)
    );
    new Trigger(m_sensor::getNoteSensed).whileTrue(
      Commands.run(()->{
        if (
          !m_vision.getXMeters().isEmpty() &&
          !m_vision.getYMeters().isEmpty() &&
          !m_vision.getZMeters().isEmpty()
        ) {
          Optional<Double> angle = m_vision.getAngleToTarget();
          if (angle.isEmpty()) {
            DashboardHelper.putString(LogLevel.Info, "Angle alignment to target", "Target Not Present.");
            // m_leds.setColor(LEDs.Color.LAVA);
          } else if (Math.abs(angle.get()) < 5) {
            DashboardHelper.putString(LogLevel.Info, "Angle alignment to target", "Aligned.");
            // m_leds.setColor(m_vision.ledsIsAligned);
          } else if (Math.abs(angle.get()) > 5) {
            // m_leds.setColor(LEDs.Color.LAVA);
            DashboardHelper.putString(LogLevel.Info, "Angle alignment to target", "Unaligned.");
          } 
        }
      })
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
          }
    }, m_leds));

    m_settings.noteSettings.resetArmEncoderTrigger.onTrue(new ResetArmEncoder(m_arm));

    new Trigger (() -> {return m_arm.armJoystickActive();}).whileTrue(
      Commands.run (
        () -> {m_arm.setSparks(m_settings.noteSettings.getManualArmJoystickValue() * .2);},
        m_arm
      )
    );

    m_settings.noteSettings.ampTrigger.onTrue(new ArmGoToGoalRotation(m_arm, Math.toRadians(95)));
    m_settings.noteSettings.storeTrigger.onTrue(new ArmGoToGoalRotation(m_arm, Math.toRadians(5.))
      .withTimeout(1.5));

    m_arm.setDefaultCommand(
      Commands.run(()->{m_arm.setSparks(0.);}, m_arm)
    );
    m_drivetrain.setDefaultCommand(m_curvatureDrive);
    // m_climber.setDefaultCommand(
    //   new Climb(m_climber, -.5)
    //   .until(()->{return m_leftClimbLimitSwitch.getAsBoolean() || m_rightClimbLimitSwitch.getAsBoolean();})
      // .andThen(new ClimbIndividual(
      //   m_climber,
      //   Math.abs(m_settings.noteSettings.getGreatestTriggerValue()),
      //   m_settings.noteSettings.getGreatestTriggerValue()>0?Side.LEFT:Side.RIGHT)
      // ).withTimeout(10.)
    // );

    // Trigger activates when a climb trigger is pushed (L2 or R2 in controller terms)
    new Trigger(m_settings.noteSettings::climberTriggered).whileTrue(new TriggerClimb(m_settings, m_climber));

    // Automatically moves the climbers down when the limit switch isnt active
    // new Trigger(()->{return m_leftClimbLimitSwitch.getAsBoolean();}).whileFalse(new ClimbIndividual(m_climber, -.7, Side.LEFT));
    // new Trigger(()->{return m_rightClimbLimitSwitch.getAsBoolean();}).whileFalse(new ClimbIndividual(m_climber, -.7, Side.RIGHT));
  }

  public void autoInit () {
    // Commands.parallel(
    //   Commands.run(()->{m_climber.setOneSide(Side.LEFT, .2);}, m_climber).until(()->{return m_climber.climberIsDown(Side.LEFT);}),
    //   Commands.run(()->{m_climber.setOneSide(Side.RIGHT, .2);}, m_climber).until(()->{return m_climber.climberIsDown(Side.RIGHT);})
    // ).schedule();;;;
  }

  public void teleopInit() {
    m_climber.resetEncoder();
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
