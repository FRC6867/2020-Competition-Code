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
 * Wrapper for the {@link GenericHID#getPOV()} function, allowing it to be used with
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
     * @return A number between 0 and 315, describing the current raw POV state.
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
        final int degrees = getPOVRaw();
        if (degrees > 180) {
            return degrees - 360;
        } else {
            return degrees;
        }
    }

    public boolean get() {
        return getPOVRaw() > 0; // We also want to ignore POV being pressed forwards.
    }
}
