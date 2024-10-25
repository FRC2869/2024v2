// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Shoots the note to the speaker automatically, Stops it after
 * Doesn't wait for rev
 */
public class ShooterAutoShootStop extends SequentialCommandGroup {
  /** Creates a new ShooterAutoShootStop. */
  public ShooterAutoShootStop() {
    addCommands(new ShooterAutoShoot(), new ShooterStop());
  }
}
