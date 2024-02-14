package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
 
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
    public FeedForwardValues feedForwardValues = new FeedForwardValues();

    /** Configurations for controller centered around drivetrain repositioning */
    public class DriveController {
        public boolean getTurnInPlaceButtonValue () { return m_driveController.getAButton();}
        public double getSpeedJoystickValue () { return MathUtil.applyDeadband(
            m_driveController.getLeftY(), deadband);
        }
        public double getRotationJoystickValue () { return MathUtil.applyDeadband( 
            m_driveController.getRightX(), deadband);
        }
        public double deadband = 0.1;

        public double normalSpeed = 0.3;
        public double turboSpeed = 1.;
    }
    /** Configurations for controller centered around note manipulation */
    public class NoteController {
        public boolean getShooterButtonValue () {return m_noteController.getAButton();}
        public boolean getIntakeButtonValue () {return m_noteController.getBButton();}
        public double getArmJoystickValue () {return MathUtil.applyDeadband( 
            m_noteController.getLeftY(), deadband);
        }
        public double deadband = 0.1;
    }
    /** To be changed later; change values based on the weight, offset, etc of the arm */
    public class FeedForwardValues {
        /** In radians, the offset of the arm */
        public final double offset = 10.;
        public final double staticGain = 1.;
        public final double gravityGain = 1.;
        public final double velocityGain = 1.;
    }
}
