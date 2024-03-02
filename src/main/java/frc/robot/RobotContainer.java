// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Intake.IntakeBasePos;
import frc.robot.commands.Intake.IntakeFloorPos;
import frc.robot.commands.Intake.IntakeSpinIn;
import frc.robot.commands.Intake.IntakeSpinOut;
import frc.robot.commands.Intake.IntakeSpinStop;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
    System.out.println("RC");
  }

  private void configureBindings() {
    Inputs.getIntakeSpinIn().onTrue(new IntakeSpinIn());
    Inputs.getIntakeBasePos().onTrue(new IntakeBasePos());
    System.out.println("rc");
    Inputs.getIntakeFloorPos().onTrue(new IntakeFloorPos());
    Inputs.getIntakeSpinOut().onTrue(new IntakeSpinOut());
    Inputs.getIntakeSpinStop().onTrue(new IntakeSpinStop());
    System.out.println("GH");
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
