/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.DriveForwardBasic;
import frc.robot.commands.TurnDegreesBasic;
import frc.robot.autonomous.AutoCommandSequence1;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import frc.robot.Constants.AutoConstants.Auto2Constants;

/**
 * yes
 */
public class AutoCommandSequence2 extends SequentialCommandGroup {
  /**
   * Creates a new AutoCommandSequence4 which drives forward, turn to the right,
   * drives to the front of the tower thing, turns (hopefully) towards it,
   * the runs shooter and stuff.
   */
  public AutoCommandSequence2(DriveTrain driveTrain, Indexer indexer, Shooter shooter) {
    super(
      new DriveForwardBasic(Auto2Constants.DRIVE_1_DISTANCE, driveTrain),
      new TurnDegreesBasic(Auto2Constants.TURN_DEGREES, driveTrain),
      new DriveForwardBasic(Auto2Constants.DRIVE_2_DISTANCE, driveTrain),
      new TurnDegreesBasic(-Auto2Constants.TURN_DEGREES, driveTrain),

      new AutoCommandSequence1(driveTrain, indexer, shooter)
    );
  }
}
