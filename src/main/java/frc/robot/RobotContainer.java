/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveForward;
import frc.robot.commands.Shoot;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.Vomit;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.DoubleButton;
import frc.robot.utils.JoystickPOV;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick m_driverGamepad = new Joystick(0);
  private final Joystick m_operatorGamepad = new Joystick(1);
  private final Camera m_camera = new Camera();

  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Shooter m_shooter = new Shooter();
  private final Indexer m_indexer = new Indexer();

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  public final JoystickButton ShooterToggleButton = new JoystickButton(m_operatorGamepad, 1);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Driving default command
    m_driveTrain.setDefaultCommand(
      new TankDrive(
        () -> -m_driverGamepad.getRawAxis(DriveTrainConstants.RIGHT_STICK_Y_AXIS_ID), // Sticks are inverted
        () -> -m_driverGamepad.getRawAxis(DriveTrainConstants.LEFT_STICK_Y_AXIS_ID),
        m_driveTrain
      )
    );

    configureButtonBindings();

    m_autoChooser.setDefaultOption("forty 2", null);
    m_autoChooser.addOption("another option", new InstantCommand());

    SmartDashboard.putData("Auto Mode", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Fine control
    new JoystickButton(m_driverGamepad, DriveTrainConstants.FINE_CONTROL_BUTTON_ID)
      .whenPressed(() -> m_driveTrain.setFineControl(true))
      .whenReleased(() -> m_driveTrain.setFineControl(false));

    // Precise turning
    final JoystickPOV m_turnDegreesPOV = new JoystickPOV(m_driverGamepad);
    m_turnDegreesPOV
      .whenPressed(new TurnDegrees(m_turnDegreesPOV.get180Degrees(), m_driveTrain));

    // Indexer control
    new JoystickButton(m_operatorGamepad, IndexerConstants.INDEXER_BUTTON_ID)
      .whenPressed(new InstantCommand(m_indexer::startIndexer))
      .whenReleased(new InstantCommand(m_indexer::stopIndexer));

    // Shooter
      new JoystickButton(m_operatorGamepad, ShooterConstants.SHOOTER_TOGGLE_BUTTON_ID)
      .whileHeld(new InstantCommand(m_shooter::enable)); // Possibly use .whenHeld()
      

    // Vomit
    new DoubleButton(m_operatorGamepad, Constants.VOMIT_BUTTON_1_ID, Constants.VOMIT_BUTTON_2_ID)
      .whenHeld(new Vomit(m_shooter, m_indexer));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return new DriveForward(-360, m_driveTrain)/*.andThen(new TurnDegrees(90, m_driveTrain))*/;
    // return new TurnDegrees(90, m_driveTrain);
    // return m_autoChooser.getSelected();
  }
}
 