// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotState;
import frc.robot.commands.LoadAutoCommand;
import frc.robot.commands.RumbleRumble;
import frc.robot.commands.SwerveResetGyro;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer m_robotContainer;

  // private LightingSubsystem lights = LightingSubsystem.getInstance();

  private Field2d field;

  // private UsbCamera camera;
  @Override
  public void robotInit() {
    // System.out.println("INIT");
    m_robotContainer = new RobotContainer();
    new SwerveResetGyro().schedule();

    Constants.timer.start();
    Constants.timer.reset();

    // new LEDCommand(LightingSetting.SCORING).schedule();
    field = new Field2d();

    // camera = CameraServer.startAutomaticCapture("cam0",0);
    // camera.setResolution(480, 360);
    // camera.setFPS(30);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumberArray("Test", Constants.testArray);
    SmartDashboard.putNumber("Timer", Timer.getMatchTime());
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("gyro", SwerveSubsystem.getInstance().getHeading().getDegrees());
    field.setRobotPose(TunerConstants.DriveTrain.getPose());
    if(Constants.currentRobotState==RobotState.DISABLED){
      ArrayList<Trajectory> trajs = m_robotContainer.getAutoTrajectory();
      if(trajs.size()>0){
        var i=0;
        for(Trajectory traj:trajs){
          i++;
          field.getObject("traj-"+i).setTrajectory(traj);
        }
      } 
    }
    SmartDashboard.putData(field);
  }

  @Override
  public void disabledInit() {
    Constants.currentRobotState = RobotState.DISABLED;
    LimelightSubsystem.getInstance().setLEDsOff();
    Inputs.getRUMBLE().whileTrue(new RumbleRumble().ignoringDisable(true));
    // lights.candle.setLEDs(0, 0, 100);
    //lights.setLights(LightingSetting.DISABLED);
    // lights.game();

    // Inputs.getLeft().onTrue(new Move(-10));
    // Inputs.getRight().onTrue(new Move(10));
  }

  @Override
  public void disabledPeriodic() {
    if (Inputs.getLoadAuto()) {
      new LoadAutoCommand().ignoringDisable(true).schedule();
      // List<List<Pose2d>> poses = m_robotContainer.getAutoTrajectory();
      // var i = 0;
      // if(poses==null){
      //   return;
      // }
      // for (List<Pose2d> pose : poses) {
      //   field.getObject("traj" + i).setPoses(pose);
      //   i++;
      // }
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    Constants.currentRobotState = RobotState.AUTON;

    // lights.setLights(LightingSetting.AUTO);
    IntakePivotSubsystem.getInstance().setBrake();
    CommandScheduler.getInstance().cancelAll();
    TunerConstants.DriveTrain.getDefaultCommand().cancel();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    Constants.currentRobotState = RobotState.TELEOP;
    // lights.setLights(LightingSetting.TELEOP);
    IntakePivotSubsystem.getInstance().setBrake();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    Constants.currentRobotState = RobotState.TEST;
    // IntakePivotSubsystem.getInstance().setBrake();
    // CommandScheduler.getInstance().cancelAll();
    // new IntakeSpeedControl().schedule();
    // new DefaultPivot().schedule();

    // lights.setLights(LightingSetting.GAME);
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** OI BRUV HAVE YOU GOT A LICENSE FOR THAT */
  public void itsTeaTimeInit() {
    System.out.println("OI BRUV HAVE YOU GOT A LICENSE FOR THAT");
  }
}
