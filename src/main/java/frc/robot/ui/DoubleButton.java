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
public class DoubleButton extends Button {
    private final GenericHID m_joystick1;
    private final GenericHID m_joystick2;

    private final int m_button1Number;
    private final int m_button2Number;

    /**
     * Creates a new {@link DoubleButton}. Is only active when both supplied buttons are pressed.
     * 
     * @param joystick1 The joystick on which the first button is located
     * @param button1Number The number of the first button, located on the first joystick
     * @param joystick2 The joystick on which the second button is located
     * @param button2Number The number of the second button, located on the second joystick
     */
    public DoubleButton(GenericHID joystick1, int button1Num, GenericHID joystick2, int button2Num) {
        m_joystick1 = joystick1;
        m_joystick2 = joystick2;
        m_button1Number = button1Num;
        m_button2Number = button2Num;
    }

    /**
     * Creates a new {@link DoubleButton}. Is only active when both supplied buttons are pressed.
     * 
     * @param joystick The joystick on which the buttons are located
     * @param button1Number The number of first button
     * @param button2Number The number of the second button
     */
    public DoubleButton(GenericHID joystick, int button1Num, int button2Num) {
        this(joystick, button1Num, joystick, button2Num);
    }


    // Returns true only if both button are pressed
    public boolean get() {
        return m_joystick1.getRawButton(m_button1Number) && m_joystick2.getRawButton(m_button2Number);
    }
}
