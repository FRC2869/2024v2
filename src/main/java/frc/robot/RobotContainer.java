// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PivotAmp;
import frc.robot.commands.PivotBase;
import frc.robot.commands.PivotFar;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Inputs.getPivotAmp().onTrue(new PivotAmp());
    Inputs.getPivotBase().onTrue(new PivotBase());
    Inputs.getPivotFar().onTrue(new PivotFar());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
