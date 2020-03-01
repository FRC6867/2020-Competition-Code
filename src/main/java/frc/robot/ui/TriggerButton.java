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
 * A special {@link Button} that only runs when the two supplied Buttons are pressed.
 */
public class TriggerButton extends Button {
    private final GenericHID m_joystick;
    private final int m_buttonNumber;

    /**
     * Creates a new {@link TriggerButton}. Is only active when supplied trigger is pressed down more than 50%.
     * 
     * @param joystick The joystick the trigger button is on
     * @param buttonNum The number of the trigger
     */
    public TriggerButton(GenericHID joystick, int buttonNum) {
        m_joystick = joystick;
        m_buttonNumber = buttonNum;
    }

    // Returns true only if both button are pressed
    public boolean get() {
        return m_joystick.getRawAxis(m_buttonNumber) >= 0.5;
    }
}
