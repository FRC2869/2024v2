// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.subsystems.PivotSubsystem;

/**
 * Sets the shooter pivot to the Far Position
 * Never Ends
 * On Interupt switches to speed control and sets speed to 0
 */
public class PivotFar extends Command {
  private PivotSubsystem pivot;

  public PivotFar() {
    pivot = PivotSubsystem.getInstance();
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {
    pivot.setPositionControl(true);
    pivot.setCurrentPosition(PositionsPivot.FAR);
    pivot.position(PivotConstants.farPosition);
  }
  @Override
  public void end(boolean i){
    pivot.setPositionControl(false);
    pivot.setSpeed(0);
  }
}
