// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PivotBase;
import frc.robot.commands.PivotFar;
import frc.robot.commands.Intake.IntakeBasePos;
import frc.robot.commands.Intake.IntakeFarPos;
import frc.robot.commands.Intake.IntakeSpinOut;
import frc.robot.commands.Intake.IntakeSpinStop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterFarShoot extends SequentialCommandGroup {
  /** Creates a new ShooterFarShoot. */
  public ShooterFarShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelRaceGroup(new IntakeFarPos(), new PivotFar(), 
    new SequentialCommandGroup(new WaitCommand(.5), 
      
      new ShooterShoot(), 
      new WaitCommand(1), new IntakeSpinOut(), 
      new WaitCommand(0.5), new ShooterStop(), 
      new IntakeSpinStop())), new ParallelRaceGroup(new IntakeBasePos(), new PivotBase(), new WaitCommand(.5)));
  }
}
