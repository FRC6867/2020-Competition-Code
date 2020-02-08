/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveTrain;

import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.DriveForward;
import frc.robot.commands.TurnDegrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick m_gamepad = new Joystick(0);

  private final DriveTrain m_driveTrain = new DriveTrain();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveTrain.setDefaultCommand( // Driving needs continuous input from sticks
      new TankDrive(
        () -> -m_gamepad.getRawAxis(DriveTrainConstants.RIGHT_STICK_Y_AXIS_ID),
        () -> -m_gamepad.getRawAxis(DriveTrainConstants.LEFT_STICK_Y_AXIS_ID),
        m_driveTrain
      )
    );

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton CLIMB_UP_BUTTON = new JoystickButton(m_gamepad, ClimbConstants.CLIMB_UP_BUTTON_ID);
    final JoystickButton CLIMB_DOWN_BUTTON = new JoystickButton(m_gamepad, ClimbConstants.CLIMB_DOWN_BUTTON_ID);
    //final JoystickButton CENTER_KRAB_BUTTON = new JoystickButton(m_gamepad, ControllerConstants.CENTER_KRAB_BUTTON);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new DriveForward(100, m_driveTrain);
    //return new TurnDegrees(90, m_driveTrain);
    return null;
  }
}
