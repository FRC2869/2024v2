// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveResetGyro extends InstantCommand {
  private Swerve swerve;

  public SwerveResetGyro() {
    swerve = Swerve.getInstance();
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    swerve.resetOdometry(new Pose2d(swerve.getPose().getTranslation(), new Rotation2d()));;
  }
}
