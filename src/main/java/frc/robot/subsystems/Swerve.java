// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Swerve extends SubsystemBase {
  private Swerve instance;
  private CommandSwerveDrivetrain swerve;

  public Swerve getInstance() {
    if (instance == null) instance = new Swerve();
    return instance;
  }
  /** Creates a new Swerve. */
  public Swerve() {
    swerve = TunerConstants.DriveTrain;
  }
}
