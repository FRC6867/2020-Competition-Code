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
    private int m_POVNumber;

    /**
     * Creates a new {@link JoystickPOV} that activates when it is moved.
     * 
     * @param joystick The {@link GenericHID} the POV is on
     * @param POVNumber The number of the POV located on the joystick
     */
    public JoystickPOV(GenericHID joystick, int POVNumber) {
        m_joystick = joystick;
        m_POVNumber = POVNumber;
    }

    /**
     * @return Current POV state. This is -1 if not pressed, or 0-360 if pressed.
     */
    public int get360Degrees() {
        return m_joystick.getPOV(m_POVNumber);
    }

    /**
     * @return A refined number between -180 and 180. Returns 0 if not active.
     */
    public int get180Degrees() {
        int rawDegrees = get360Degrees();
        int degrees = rawDegrees * 180;
        if (rawDegrees > 180) {
            degrees = -degrees;
        } else if (rawDegrees < 0) {
            degrees = 0;
        }

        return degrees;
    }

    public boolean get() {
        return get360Degrees() != -1;
    }
}
