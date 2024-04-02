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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

/**
 * Recieves data from the limelight.
 * @author Summer Singh
 */
public class LimelightSubsystem extends SubsystemBase {
  
  private static LimelightSubsystem instance;

  private static CommandSwerveDrivetrain swerve;

  public enum LightingState{
    OFF, ON, BLINK
  }

  private LightingState currentState = LightingState.OFF;

  private NetworkTable table;
  private NetworkTableEntry botPose;
  // private CommandSwerveDrivetrain swerve;

  private String ll1;

  private String ll2;

  /** Gets the limelight object. */
  public static LimelightSubsystem getInstance() {
    if (instance == null) instance = new LimelightSubsystem();
    return instance;
  }

  /**
   * Creates a new LimelightSubsystem.
   */
  public LimelightSubsystem() {
    ll1 = "limelight-ankur";
    ll2 = "limelight-arsh";
    swerve = TunerConstants.DriveTrain;
    table = NetworkTableInstance.getDefault().getTable(ll1);
    botPose = table.getEntry("botpose_wpiblue");
    setLEDsOff();
  }

  /** @return double array containing x,y,z,roll,pitch,yaw */
  public double[] getArray() {
    return botPose.getDoubleArray(new double[6]);
  }

  /**
   * @return position based on limelight or null if no point found
   */
  public Pose2d getLimelightPose() {
    try {
      double[] array = getArray();
      if(array[0]==0) return null;
      return new Pose2d(new Translation2d(array[0], array[1]), SwerveSubsystem.getInstance().getHeading());
    }
    catch(Exception e) {return null;}
  }

  public void setLEDsOn(){
    LimelightHelpers.setLEDMode_ForceOn(ll1);
    LimelightHelpers.setLEDMode_ForceOn(ll2);
    currentState = LightingState.ON;
  }

  public void setLEDsOff(){
    LimelightHelpers.setLEDMode_ForceOff(ll1);
    LimelightHelpers.setLEDMode_ForceOff(ll2);
    currentState = LightingState.OFF;
  }

  public void setLEDsBlink(){
    LimelightHelpers.setLEDMode_ForceBlink(ll1);
    LimelightHelpers.setLEDMode_ForceBlink(ll2);
    currentState = LightingState.BLINK;
  }

  public LightingState getCurrentLightingState(){
    return currentState;
  }
  public double[] clean(double[] a) {
    double[] array = new double[a.length];
    for (int i = 0; i < a.length; i++)
      array[i] = ((double)(int)(a[i] * 100 + .5)/100);
    return array;
  }

  @Override
  public void periodic() {
    if (getArray()[0] != 0)SmartDashboard.putNumberArray("limelight bot pose", clean(getArray()));
    try{
      if(SwerveSubsystem.getInstance().getRobotRelativeSpeeds().vxMetersPerSecond<0.25||SwerveSubsystem.getInstance().getRobotRelativeSpeeds().vyMetersPerSecond<0.25)
        swerve.addVisionMeasurement(getLimelightPose(), Timer.getFPGATimestamp());
    }
    catch(Exception e) {}
  }
}
