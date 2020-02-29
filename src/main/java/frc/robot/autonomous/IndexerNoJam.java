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

import frc.robot.subsystems.Indexer;

/**
 * A command group that runs the indexer forwads and backwards for specified times.
 */
public class IndexerNoJam extends SequentialCommandGroup {
  /**
   * Creates a new IndexerNoJamSequence, which runs the indexer motor forward and backward for
   * specified runTime and vomitTime seconds. It should be noted that the indexer's
   * {@link Indexer.vomit()} command will be called to reverse. The indexer will be left running
   * backwards when the sequence ends.
   * 
   * @param runTime The amount of time, in seconds, the indexer should be running
   * forwards
   * @param vomitTime the amount of time, in seconds, the indexer should be
   * running backwards
   * @param stopAtEnd Whether the indexer should be stopped at the end
   * @param indexer The Indexer subsystem
   */
  public IndexerNoJam(double runTime, double vomitTime, boolean stopAtEnd, Indexer indexer) {
    super(
      new StartEndCommand(indexer::startIndexer, indexer::vomit) // Run and stop
        .withTimeout(runTime),
      new WaitCommand(vomitTime) // Run this long in vomit
    );

    if (stopAtEnd) { // Add a stop indexer if stopAtEnd
      this.addCommands(
        new InstantCommand(indexer::stopIndexer)
      );
    }
    
    addRequirements(indexer);
  }

  /**
   * Creates a new IndexerNoJamSequence, which runs the indexer motor forward and backward for
   * specified runTime and vomitTime seconds. It should be noted that the indexer's
   * {@link Indexer.vomit()} command will be called to reverse. The indexer will be left running
   * backwards when the sequence ends.
   * 
   * @param runTime The amount of time, in seconds, the indexer should be running
   * forwards
   * @param vomitTime the amount of time, in seconds, the indexer should be
   * running backwards
   * @param indexer The Indexer subsystem
   */
  public IndexerNoJam(double runTime, double vomitTime, Indexer indexer) {
    this(runTime, vomitTime, false, indexer);
  }
}
