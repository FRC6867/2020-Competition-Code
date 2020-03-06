/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.autonomous.*;
import frc.robot.ui.*;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Joysticks
  private final Joystick m_driverGamepad = new Joystick(0);
  private final Joystick m_operatorGamepad = new Joystick(1);

  // Subsystems
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Indexer m_indexer = new Indexer();
  private final Shooter m_shooter = new Shooter();
  private final Vision m_vision = new Vision();

  // Other
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    // Auto command chooser
    m_autoChooser.setDefaultOption("Sequence Number 1",
      new AutoCommandSequence1(m_driveTrain, m_indexer, m_shooter));
    m_autoChooser.addOption("Sequence Number 2", 
      new AutoCommandSequence2(m_driveTrain, m_indexer, m_shooter));
    m_autoChooser.addOption("Turn 90 degrees", new TurnDegrees(90, m_driveTrain));
    m_autoChooser.addOption("Drive Forwards 100 inches", new DriveForward(100, m_driveTrain));
    m_autoChooser.addOption("Do nothing", null);

    SmartDashboard.putData("Auto Mode", m_autoChooser);
    SmartDashboard.setPersistent("Auto Mode");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // DriveTrain buttons
    final JoystickButton m_fineControlButton = new JoystickButton(m_driverGamepad, DriveTrainConstants.FINE_CONTROL_BUTTON_ID);

    // Indexer buttons
    final JoystickButton m_indexerRightForwardButton = new JoystickButton(m_operatorGamepad, IndexerConstants.INDEXER_RIGHT_FORWARD_BUTTON_ID);
    final TriggerButton m_indexerRightBackwardButton = new TriggerButton(m_operatorGamepad, IndexerConstants.INDEXER_RIGHT_BACKWARD_BUTTON_ID);
    final JoystickButton m_indexerLeftForwardButton = new JoystickButton(m_operatorGamepad, IndexerConstants.INDEXER_LEFT_FORWARD_BUTTON_ID);
    final TriggerButton m_indexerLeftBackwardButton = new TriggerButton(m_operatorGamepad, IndexerConstants.INDEXER_LEFT_BACKWARD_BUTTON_ID);

    // Shooter buttons
    final JoystickButton m_shooterSpinButton = new JoystickButton(m_operatorGamepad, ShooterConstants.SHOOTER_TOGGLE_BUTTON_ID);
    final JoystickButton m_feederButton = new JoystickButton(m_operatorGamepad, ShooterConstants.FEEDER_BUTTON_ID);
    final DoubleButton m_vomitButton = new DoubleButton(m_operatorGamepad, ShooterConstants.VOMIT_BUTTON_1_ID, ShooterConstants.VOMIT_BUTTON_2_ID);

    // Driving default command
    m_driveTrain.setDefaultCommand(
      new TankDrive(
        () -> -m_driverGamepad.getRawAxis(DriveTrainConstants.RIGHT_STICK_Y_AXIS_ID), // Sticks are inverted
        () -> -m_driverGamepad.getRawAxis(DriveTrainConstants.LEFT_STICK_Y_AXIS_ID),
        m_driveTrain
      )
    );

    // Fine control
    m_fineControlButton
      .toggleWhenPressed(new StartEndCommand(
        () -> m_driveTrain.setFineControl(true),
        () -> m_driveTrain.setFineControl(false)
      ));
    
    // Indexer control.
    m_indexer.setDefaultCommand(
      new IndexerControl(
        m_indexerRightForwardButton,
        m_indexerRightBackwardButton,
        m_indexerLeftForwardButton,
        m_indexerLeftBackwardButton,
        m_indexer
      )
    );

    // Shooter spin-up
    m_shooterSpinButton
      .toggleWhenPressed(new StartEndCommand(m_shooter::enable, m_shooter::disable, m_shooter), false);

    // Shooter shoot (feeder)
    m_feederButton
      .whenHeld(new FeedShooter(m_shooter, m_indexer));

    // Vomit
    m_vomitButton
      .whenHeld(new Vomit(m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}