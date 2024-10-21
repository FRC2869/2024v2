// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.subsystems.PivotSubsystem;

/**
 * Sets the shooter pivot to the Close Position
 * Never Ends
 * On Interupt switches to speed control and sets speed to 0
 */
public class PivotClose extends Command {
  private PivotSubsystem pivot;

  public PivotClose() {
    pivot = PivotSubsystem.getInstance();
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

boolean hasRun = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!hasRun){
      hasRun = true;
      System.out.println(this.getName()+ " Start:"+Constants.timer.get());
    }
    pivot.setPositionControl(true);
    pivot.setCurrentPosition(PositionsPivot.CLOSE);
    pivot.position(PivotConstants.closePosition);
  }
  @Override
  public void end(boolean i){
    System.out.println(this.getName()+ " End:"+Constants.timer.get());
    pivot.setPositionControl(false);
    pivot.setSpeed(0);
  }
}
