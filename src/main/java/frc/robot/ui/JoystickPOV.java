/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ui;

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
     * @return A number betwenen 0 and 315, describing the current raw POV state.
     * Returns -1 if not active.
     */
    public int getPOVRaw() {
        return m_joystick.getPOV();
    }

    /**
     * @return A refined number between -135 and 180, describing the current POV state.
     * Returns 0 if not active.
     */
    public int getPOV180() {
        int degrees = getPOVRaw();
        if (degrees < 0) {
            return 0;
        } else {
            return degrees - 180;
        }
    }

    // In our use case, we do not want to activate even if POV is pressed forward.
    public boolean get() {
        return getPOVRaw() > 0;
    }
    
    /**
    // See above. This might be faster though.
    public boolean get() {
        return getRawPOV() != -1;
    }
    */
}
