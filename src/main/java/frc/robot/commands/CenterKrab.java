/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.Krab;

import frc.robot.Constants.PIDConstants;


public class CenterKrab extends PIDCommand {
  /**
   * Creates a new CenterKrab command.
   *
   * !! Curently not finished
   * 
   * @param krab The Krab subsystem
   */
  public CenterKrab(Krab krab) {
    super(
        new PIDController(PIDConstants.KRAB_P, PIDConstants.KRAB_I, PIDConstants.KRAB_D),
        krab::getPosition,
        0,
        // This uses the output
        output -> krab.slide(output),
        krab
    );
    
    getController().setTolerance(200);
    
    addRequirements(krab);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return getController().atSetpoint() // Disabled for tuning
    return false;
  }
}
