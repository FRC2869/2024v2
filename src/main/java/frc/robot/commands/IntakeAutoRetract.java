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

/**
 *  Sequential Command for putting the intake up, 
 *  stopping intake, and then going back to base position. 
 *  This class will be used by autonomous commands
 */
public class IntakeAutoRetract extends SequentialCommandGroup {
  /** Creates a new IntakeAutoRetract. */
  public IntakeAutoRetract() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IntakeSpinStop(), new ParallelRaceGroup(new IntakeBasePos(), new SequentialCommandGroup(new WaitCommand(.1), new IntakeWaitPosition()))
      );
  }
}
