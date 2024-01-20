package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

public class Settings {
    private XboxController m_driveController, m_noteController;
    Settings (XboxController driveController, XboxController noteController){
        m_driveController = driveController;
        m_noteController = noteController;
    }

    public class DriveController {          
        public boolean getTurnInPlaceButton () { return m_driveController.getAButtonPressed();}
        public double getSpeedJoystick () { return MathUtil.applyDeadband(
            m_driveController.getLeftY(), deadband);
        }
        public double getRotationJoystick () { return MathUtil.applyDeadband( 
            m_driveController.getRightX(), deadband);
        }
        public double deadband = 0.1;
    }
    public class NoteController {
        public boolean getShooterButton () {return m_noteController.getAButtonPressed();}
        public double getArmJoystick () {return MathUtil.applyDeadband( 
            m_noteController.getLeftY(), deadband);
        }
        public double deadband = 0.1;
    }
}
