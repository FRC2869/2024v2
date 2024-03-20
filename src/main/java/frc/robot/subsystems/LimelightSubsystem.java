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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;

/**
 * Recieves data from the limelight.
 * @author Summer Singh
 */
public class LimelightSubsystem extends SubsystemBase {
  
  private static LimelightSubsystem instance;

  private NetworkTable table;
  private NetworkTableEntry botPose;
  private CommandSwerveDrivetrain swerve;

  /** Gets the limelight object. */
  public static LimelightSubsystem getInstance() {
    if (instance == null) instance = new LimelightSubsystem();
    return instance;
  }

  /**
   * Creates a new LimelightSubsystem.
   */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("Pipeline_Name");
    botPose = table.getEntry("botpose");
  }

  /** @return double array containing x,y,z,roll,pitch,yaw */
  public double[] getArray() {
    return botPose.getDoubleArray(new double[6]);
  }

  /**
   * @return position based on limelight
   */
  public Pose2d getLimelightPose() {
    try {
      double[] array = getArray();
      return new Pose2d(new Translation2d(array[0], array[1]), new Rotation2d(array[5]));
    }
    catch(Exception e) {return null;}
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumberArray("limelight bot pose", getArray());
    swerve.addVisionMeasurement(getLimelightPose(), 0);
  }
}
