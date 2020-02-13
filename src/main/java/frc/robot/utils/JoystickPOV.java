/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Wrapper for the {@link GenericHID#getPOV(int)} functions, allowing it to be used with
 * {@link Button} methods.
 */
public class JoystickPOV extends Button {
    private GenericHID m_joystick;

    /**
     * Creates a new {@link JoystickPOV} that activates when it is moved.
     * 
     * @param joystick The {@link GenericHID} the POV is on
     */
    public JoystickPOV(GenericHID joystick) {
        m_joystick = joystick;
    }

    /**
     * @return Current raw POV state. This is -1 if not pressed, or 0-315 if pressed.
     */
    public int getRawPOV() {
        return m_joystick.getPOV();
    }

    /**
     * @return A refined number between -180 and 180, describing the current POV state.
     * Returns 0 if not active.
     */
    public int get180Degrees() {
        int rawDegrees = getRawPOV();
        int degrees = rawDegrees % 180;
        if (rawDegrees > 180) {
            degrees = -degrees;
        } else if (rawDegrees < 0) {
            degrees = 0;
        }

        return degrees;
    }

    // In our use case, we do not want to activate if POV is pressed forward.
    public boolean get() {
        return get180Degrees() != 0;
    }
    
    /**
    // See above. This might be faster though.
    public boolean get() {
        return getRawPOV() != -1;
    }
    */
}
