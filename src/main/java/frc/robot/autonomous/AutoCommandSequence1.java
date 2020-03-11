/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.RepeatCommand;
import frc.robot.commands.TimedDrive;
import frc.robot.commands.FeedShooter;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import frc.robot.Constants.AutoConstants.Auto1Constants;

public class AutoCommandSequence1 extends SequentialCommandGroup {
  /**
   * Creates a new AutoCommandSequence3.
   */
  public AutoCommandSequence1(DriveTrain driveTrain, Indexer indexer, Shooter shooter) {
    super(
      new InstantCommand(shooter::enable, shooter),
      new TimedDrive(Auto1Constants.DRIVE_TIME, Auto1Constants.DRIVE_SPEED, driveTrain),

      new WaitCommand(Auto1Constants.SHOOT_DELAY),
      parallel( // Run feeder and indexer
        new FeedShooter(shooter, indexer)
          .withTimeout(Auto1Constants.SHOOT_TIME),
        
        //new InstantCommand(() -> indexer.run(1, -1), indexer),
        //new WaitCommand(0.25),
        new StartEndCommand(indexer::runAll, indexer::stop, indexer)
          .withTimeout(Auto1Constants.SHOOT_TIME)

        // No need anymore
        // new RepeatCommand(
        //   new IndexerNoJam(1.5, 0.5, indexer),
        //   5
        // )
      )
      .andThen(shooter::disable, shooter)
    );
  }
}
