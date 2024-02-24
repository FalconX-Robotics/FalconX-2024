package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
 
public class Settings {
    // Access Controllers
    private XboxController m_driveController, m_noteController;
    Settings (XboxController driveController, XboxController noteController){
        m_driveController = driveController;
        m_noteController = noteController;
                    m_driveController.setRumble(RumbleType.kBothRumble, 10);

    }
    public DriveController driveController = new DriveController();
    public NoteController noteController = new NoteController();

    public static double curveInput(double input) {
        
            return Math.pow(input, 2) * Math.signum(input);
    }

    /** Configurations for controller centered around drivetrain repositioning */
    public class DriveController {
        public boolean getTurnInPlaceButtonValue () { 
                        m_driveController.setRumble(RumbleType.kBothRumble, 10);
return m_driveController.getAButtonPressed();}
        public double getSpeedJoystickValue () { 
            m_driveController.setRumble(RumbleType.kBothRumble, 10);
            return MathUtil.applyDeadband(
            -m_driveController.getLeftY(), deadband);
        }
        public double getRotationJoystickValue () {
                        m_driveController.setRumble(RumbleType.kBothRumble, 1);
 return MathUtil.applyDeadband( 
            m_driveController.getRightX(), deadband);
        }

        

        public double deadband = 0.1;

        public double normalSpeed = 0.5;
        public double turboSpeed = 1.;
    }
    /** Configurations for controller centered around note manipulation */
    public class NoteController {
        public boolean getShooterButtonValue () {return m_noteController.getAButtonPressed();}
        public double getArmJoystickValue () {return MathUtil.applyDeadband( 
            -m_noteController.getLeftY(), deadband);
        }
        public double deadband = 0.1;
    }
}
