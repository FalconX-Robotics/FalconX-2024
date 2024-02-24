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

    // Do not remove. or else
    public DriveController driveController = new DriveController();
    public NoteController noteController = new NoteController();

    public static double curveInput(double input) {
        
            return Math.pow(input, 2) * Math.signum(input);
    }
    public FeedForwardValues feedForwardValues = new FeedForwardValues();

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
        public Button getTurboButton () {return XboxController.Button.kRightBumper;}
        public Button getTurnInPlaceButton () {return XboxController.Button.kLeftBumper;}

        public double deadband = 0.1;
        public double normalSpeed = 0.3;
        public double turboSpeed = 1.;
    }
    /** Configurations for controller centered around note manipulation */
    public class NoteController {
        public Button getShooterButton () {return XboxController.Button.kA;}
        public Button getIntakeButton  () {return XboxController.Button.kB;}
        public Button getIndexButton   () {return XboxController.Button.kB;}
        public Button getReverseButton () {return XboxController.Button.kRightBumper;}

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
