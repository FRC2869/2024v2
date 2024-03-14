// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotAmp extends Command {
  private PivotSubsystem pivot;

  public PivotAmp() {
    pivot = PivotSubsystem.getInstance();
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {
    pivot.setPositionControl(true);
    pivot.setCurrentPosition(PositionsPivot.AMP);
    pivot.position(PivotConstants.ampPosition);
    System.out.println("AMP");
  }
  @Override
  public void end(boolean i){
    pivot.setPositionControl(false);
    pivot.setSpeed(0);
  }
}
