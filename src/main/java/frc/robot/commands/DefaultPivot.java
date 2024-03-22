// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Inputs;
import frc.robot.subsystems.PivotSubsystem;

/**
 * Sets the shooter pivot to the Speed Control using Inputs.getPivotOverride()
 * Never Ends
 * On Interupt stays on speed control and sets speed to 0
 */
public class DefaultPivot extends Command {
  private PivotSubsystem pivot;

  /** Creates a new DefaultPivot. */
  public DefaultPivot() {
    pivot = PivotSubsystem.getInstance();
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // pivot.setSpeed(Inputs.getPivotOverride());
    pivot.setPositionControl(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setSpeed(0);
  }

}
