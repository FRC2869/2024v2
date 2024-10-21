// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PivotClose;
import frc.robot.commands.Intake.IntakeClosePos;
import frc.robot.commands.Intake.IntakeSpinOut;

/**
 * Shoots the note to the speaker automatically, Stops it after
 * Doesn't wait for rev
 */
public class ShooterAutoShootAuton extends SequentialCommandGroup {
  /** Creates a new ShooterAutoShootStop. */
  public ShooterAutoShootAuton() {
    addCommands(new IntakeClosePos(), new PivotClose(), new SequentialCommandGroup(new ShooterShoot(), new IntakeSpinOut()));
  }
}
