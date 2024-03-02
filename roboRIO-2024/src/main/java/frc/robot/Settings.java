package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
 
public class Settings {
    // Access Controllers
    private XboxController m_driveController,  m_noteController;
    public DriveSettings driveSettings;
    public NoteSettings noteSettings;
    Settings (XboxController driveController, XboxController noteController){
        m_driveController = driveController;
        m_noteController = noteController;
        // m_driveController.setRumble(RumbleType.kBothRumble, 10);
        // Do not remove. Or else :)
        driveSettings = new DriveSettings();
        noteSettings = new NoteSettings();
    }

    public static double curveInput(double input) {
        return Math.pow(input, 2) * Math.signum(input);
    }

    /** Configurations for controller centered around drivetrain repositioning */
    public class DriveSettings {
        public double getSpeedJoystickValue () { 
            // m_driveController.setRumble(RumbleType.kBothRumble, 10); //?
            return MathUtil.applyDeadband(
                -m_driveController.getLeftY(), deadband
            );
        }
        public double getRotationJoystickValue () {
            // m_driveController.setRumble(RumbleType.kBothRumble, 1);
            return MathUtil.applyDeadband( 
                m_driveController.getRightX(), deadband
            );
        }

        public Trigger turboModeTrigger   = new JoystickButton(m_driveController, Button.kRightBumper.value);
        public Trigger turnInPlaceTrigger = new JoystickButton(m_driveController, Button.kLeftBumper.value);
        
        public double deadband = 0.1;
        public double normalSpeed = 0.3;
        public double turboSpeed = 1.;
    }
    /** Configurations for controller centered around note manipulation */
    public class NoteSettings { 

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
