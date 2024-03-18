// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeBasePos;
import frc.robot.commands.Intake.IntakeClosePos;
import frc.robot.commands.Intake.IntakeSpinStop;
import frc.robot.commands.Intake.IntakeWaitPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * A sequnetial command that stops intake, and moves it back to it's original position.
 */
public class IntakeAutoRetract extends SequentialCommandGroup {
  /** Creates a new IntakeAutoRetract. */
  public IntakeAutoRetract() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IntakeSpinStop(), new ParallelRaceGroup(new IntakeClosePos(), new SequentialCommandGroup(new WaitCommand(.1), new IntakeWaitPosition()))
      );
  }
}
