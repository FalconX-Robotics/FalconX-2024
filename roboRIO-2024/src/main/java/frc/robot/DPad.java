// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * A {@link Trigger} that gets its state from a {@link GenericHID}.
 *
 * <p>This class is based on {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}
 */
public class DPad extends Trigger {
  public enum Direction {
    NONE(-1),
    UP(0),
    UP_RIGHT(45),
    RIGHT(90),
    DOWN_RIGHT(135),
    DOWN(180),
    DOWN_LEFT(225),
    LEFT(270),
    UP_LEFT(315);

    public final int value;
    Direction(int value){
        this.value = value;
    }
    public double getValue(){
        return value;
    }
  }
    /**
   * Creates a POV/D-Pad button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param direction The direction of the Controller as an enum
   */
  public DPad(GenericHID joystick, int direction) {
    super(() -> joystick.getPOV() == direction);
    // requireNonNullParam(joystick, "joystick", "DPad");
  }
}
