// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PivotClose;
import frc.robot.commands.Intake.IntakeClosePos;
import frc.robot.commands.Intake.IntakeSpinOut;
import frc.robot.commands.Intake.IntakeSpinStop;

/**
 * Shoots the note automatically, waiting for rev up
 */
public class ShooterAutoShootTeleop extends ParallelRaceGroup {
  /** Creates a new ShooterAutoShoot. */
  public ShooterAutoShootTeleop() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IntakeClosePos(), new PivotClose(), new SequentialCommandGroup(new ShooterShoot(), new ParallelCommandGroup(new ShooterRevWait(), new WaitCommand(2)), new IntakeSpinOut(), new WaitCommand(1), new IntakeSpinStop(), new ShooterStop()));
    // addCommands(new AimAtSpeaker(), new WaitCommand(1),new ShooterShoot(), new WaitCommand(1), new IntakeSpinOut(), new WaitCommand(1), new ShooterStop(), new IntakeSpinStop());
  }
}
