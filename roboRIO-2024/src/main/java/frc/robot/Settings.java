package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
 
public class Settings {
    // Access Controllers
    private XboxController m_driveController, m_noteController;
    Settings (XboxController driveController, XboxController noteController){
        m_driveController = driveController;
        m_noteController = noteController;
    }

    // Do not remove. or else
    public DriveController driveController = new DriveController();
    public NoteController noteController = new NoteController();

    /** Configurations for controller centered around drivetrain repositioning */
    public class DriveController {
        // TODO replace this
        public double getSpeedJoystickValue () { return MathUtil.applyDeadband(
            m_driveController.getLeftY(), deadband);
        }
        public double getRotationJoystickValue () { return MathUtil.applyDeadband( 
            m_driveController.getRightX(), deadband);
        }

        public Trigger turboModeTrigger   = new JoystickButton(m_driveController, Button.kRightBumper.value);
        public Trigger turnInPlaceTrigger = new JoystickButton(m_driveController, Button.kLeftBumper.value);
        
        public double deadband = 0.1;
        public double normalSpeed = 0.3;
        public double turboSpeed = 1.;
    }
    /** Configurations for controller centered around note manipulation */
    public class NoteController {
        public Trigger shooterChargeTrigger   = new JoystickButton(m_noteController, Button.kA.value);
        public Trigger shooterFireTrigger     = new JoystickButton(m_noteController, Button.kX.value);
        public Trigger intakeTrigger          = new JoystickButton(m_noteController, Button.kB.value);
        public Trigger reverseTrigger         = new JoystickButton(m_noteController, Button.kRightBumper.value);
        
        public double getArmJoystickValue () {return MathUtil.applyDeadband( 
            m_noteController.getLeftY(), deadband);
        }
        public double deadband = 0.1;
    }
}
