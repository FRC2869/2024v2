// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeSpinOut;
import frc.robot.commands.Intake.IntakeSpinStop;

/**
 * Automatically shoots the shooter, doesn't stop it
 * Doesn't Wait for rev
 */
public class ShooterAutoAngleShoot extends ParallelRaceGroup {
  /** Creates a new ShooterAutoShoot. */
  public ShooterAutoAngleShoot() {

    addCommands(new AutoAimIntake(), new SequentialCommandGroup(new ShooterShoot(), new IntakeSpinOut(), new WaitCommand(0.25), new IntakeSpinStop()));
    // addCommands(new AimAtSpeaker(), new WaitCommand(1),new ShooterShoot(), new WaitCommand(1), new IntakeSpinOut(), new WaitCommand(1), new ShooterStop(), new IntakeSpinStop());
  }
}
