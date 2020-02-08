/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.DriveTrain;

import frc.robot.Constants.PIDConstants;

public class TurnDegrees extends PIDCommand {
  /**
   * Creates a new TurnDegrees command.
   * 
   * @param degrees The amount of degrees to turn. Positive - right, negative - left
   * @param driveTrain The {@link DriveTrain} subsystem
   */
  public TurnDegrees(int degrees, DriveTrain driveTrain) {
    super(
      new PIDController(PIDConstants.TURN_P, PIDConstants.TURN_I, PIDConstants.TURN_D),
      driveTrain::getHeading,
      degrees,
      output -> driveTrain.drive(-output, output),
      driveTrain
    );

    
    driveTrain.reset(); // Make sure we are in relative position
    setPID();
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(0.5); // 0.5 degrees inaccuracy allowed

    addRequirements(driveTrain);
  }

  private void setPID() {
    SmartDashboard.putNumber("Turn P", getController().getP());
    SmartDashboard.putNumber("Turn I", getController().getI());
    SmartDashboard.putNumber("Turn D", getController().getD());
    
    getController().setPID(
      SmartDashboard.getNumber("Turn P", PIDConstants.TURN_P),
      SmartDashboard.getNumber("Turn I", PIDConstants.TURN_I),
      SmartDashboard.getNumber("Turn D", PIDConstants.TURN_D)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
