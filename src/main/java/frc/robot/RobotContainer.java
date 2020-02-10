/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.utils.DoubleButton;
import frc.robot.utils.JoystickPOV;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.DriveForward;
import frc.robot.commands.Shoot;
import frc.robot.commands.Vomit;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;

import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick m_gamepad1 = new Joystick(0);
  private final Joystick m_gamepad2 = new Joystick(1);

  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Shooter m_shooter = new Shooter();
  private final Indexer m_indexer = new Indexer();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveTrain.setDefaultCommand( // Driving needs continuous input from sticks
      new TankDrive(
        () -> -m_gamepad1.getRawAxis(DriveTrainConstants.RIGHT_STICK_Y_AXIS_ID),
        () -> -m_gamepad1.getRawAxis(DriveTrainConstants.LEFT_STICK_Y_AXIS_ID),
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
    final JoystickButton m_releaseClimbButton = new JoystickButton(m_gamepad2, ClimbConstants.RELEASE_CLIMB_BUTTON_ID);
    final JoystickButton m_climbUpButton = new JoystickButton(m_gamepad2, ClimbConstants.CLIMB_UP_BUTTON_ID);
    final JoystickButton m_shootButton = new JoystickButton(m_gamepad2, ShooterConstants.SHOOTER_TOGGLE_BUTTON_ID);
    final JoystickButton m_fineControlButton = new JoystickButton(m_gamepad1, DriveTrainConstants.FINE_CONTROL_BUTTON_ID);
    final JoystickButton m_indexerButton = new JoystickButton(m_gamepad2, IndexerConstants.INDEXER_BUTTON_ID);
    final JoystickPOV m_turnDegreesPOV = new JoystickPOV(m_gamepad1);
    final DoubleButton m_vomitButton = new DoubleButton(m_gamepad2, Constants.VOMIT_BUTTON_1_ID, Constants.VOMIT_BUTTON_2_ID);
    /**
    // Shooter bindings
    m_shootButton.toggleWhenActive(new Shoot(m_shooter, m_indexer));
    //m_shootButton.whenHeld(new Shoot(m_shooter, m_indexer));

    // Precise Turning
    m_turnDegreesPOV.whenPressed(new TurnDegrees(
      m_turnDegreesPOV.get180Degrees(), m_driveTrain
    ));

    // Indexer control inline command
    m_indexerButton.whenPressed(new InstantCommand(m_indexer::startIndexer))
    .whenReleased(new InstantCommand(m_indexer::stopIndexer));

    // Fine control inline command
    m_fineControlButton.whenPressed(
      new InstantCommand(
        () -> m_driveTrain.setFineControl(true)
      )
    ).whenReleased(
      new InstantCommand(
        () -> m_driveTrain.setFineControl(false)
      )
    );

    // Vomit function
    m_vomitButton.whenHeld(new Vomit(m_shooter, m_indexer));
    */
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
