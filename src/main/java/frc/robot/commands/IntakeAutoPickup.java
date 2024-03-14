// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeFloorPos;
import frc.robot.commands.Intake.IntakeSpinIn;
import frc.robot.commands.Intake.IntakeWaitPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 *  Sequential Command for putting the intake down, 
 *  intaking the note, and then going back to base position. 
 *  This class will be used by autonomous commands
 *  @TODO Program this to work with a button?
 * 
 */
public class IntakeAutoPickup extends SequentialCommandGroup {
  /** Creates a new IntakeAutoPickup. */
  public IntakeAutoPickup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeSpinIn(), 
      new ParallelRaceGroup(new IntakeFloorPos(), new IntakeWaitPosition())
    );
      // new WaitCommand(1), 
      // new ParallelRaceGroup(new IntakeBasePos(), new IntakeWaitPosition()),
      // new IntakeSpinStop());
  }
}
