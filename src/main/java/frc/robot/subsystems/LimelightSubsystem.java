// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;

public class LimelightSubsystem extends SubsystemBase {
  
  private static LimelightSubsystem instance;

  private NetworkTable table;
  private NetworkTableEntry botPose;
  private CommandSwerveDrivetrain swerve;

  public static LimelightSubsystem getInstance() {
    if (instance == null) instance = new LimelightSubsystem();
    return instance;
  }

  //tid primary april tag
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    botPose = table.getEntry("botpose");
  }

  public double[] getArray() {
    return botPose.getDoubleArray(new double[6]);
  }

  //0, 1, 4
  public Pose2d getLimelightPose() {
    try {
      double[] array = getArray();
      return new Pose2d(new Translation2d(array[0], array[1]), new Rotation2d(array[4]));
    }
    catch(Exception e) {return null;}
  }


  @Override
  public void periodic() {
    swerve.addVisionMeasurement(getLimelightPose(), 0);
  }
}
