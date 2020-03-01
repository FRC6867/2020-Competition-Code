/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.RepeatCommand;
import frc.robot.commands.TimedDrive;
import frc.robot.commands.Shoot;
import frc.robot.commands.IndexerNoJam;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import frc.robot.Constants.AutoConstants.Auto3Constants;

public class AutoCommandSequence3 extends SequentialCommandGroup {
  /**
   * Creates a new AutoCommandSequence3.
   */
  public AutoCommandSequence3(DriveTrain driveTrain, Indexer indexer, Shooter shooter) {
    super(
      new InstantCommand(shooter::enable, shooter),
      new TimedDrive(Auto3Constants.DRIVE_TIME, 0.5, driveTrain),

      parallel( // Run feeder and indexer
        new Shoot(shooter, indexer)
          .withTimeout(Auto3Constants.SHOOT_TIME),
        
        new StartEndCommand(indexer::startIndexer, indexer::stopIndexer, indexer)
          .withTimeout(Auto3Constants.SHOOT_TIME)

        // new RepeatCommand( // No need anymore
        //   new IndexerNoJam(1.5, 0.5, indexer),
        //   5
        // )
      )
      .andThen(shooter::disable, shooter)
    );
  }
}
