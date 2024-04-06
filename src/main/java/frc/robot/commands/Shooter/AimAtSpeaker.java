// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AimAtSpeaker extends InstantCommand {
  private PivotSubsystem pivot = PivotSubsystem.getInstance();
  private IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();
  private CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain;
  /** Creates a new ShootAtSpeaker. */
  public AimAtSpeaker() {
    addRequirements(pivot, intakePivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {
    pivot.setPositionControl(true);
    pivot.position((180 * (swerve.getAngle()/Math.PI)));
    intakePivot.setPivotPos(180 * swerve.getIntakeAngle()/Math.PI/Constants.PivotConstants.intakeGearMultiplier);
  }
}
