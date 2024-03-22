// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants.LightingSetting;
import frc.robot.commands.SwerveResetGyro;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer m_robotContainer;
  
  private LightingSubsystem lights = LightingSubsystem.getInstance();

  private Field2d field;

  private UsbCamera camera;
  @Override
  public void robotInit() {
  //  System.out.println("INIT");
    m_robotContainer = new RobotContainer();
    new SwerveResetGyro().schedule();
    //new LEDCommand(LightingSetting.SCORING).schedule();
    field = new Field2d();

    
	  camera = CameraServer.startAutomaticCapture("cam0",0);
    camera.setResolution(480, 360);
    camera.setFPS(30);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    SmartDashboard.putNumber("gyro", Swerve.getInstance().getHeading().getDegrees());
    field.setRobotPose(TunerConstants.DriveTrain.getPose());
    SmartDashboard.putData(field);
  }

  @Override
  public void disabledInit() {
    //lights.setLights(LightingSetting.DISABLED);
    lights.game();
    
    // Inputs.getLeft().onTrue(new Move(-10));
    // Inputs.getRight().onTrue(new Move(10));
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    lights.setLights(LightingSetting.AUTO);
    IntakePivotSubsystem.getInstance().setBrake();
    CommandScheduler.getInstance().cancelAll();
    TunerConstants.DriveTrain.getDefaultCommand().cancel();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  
  @Override
  public void autonomousPeriodic() {}
  
  @Override
  public void autonomousExit() {}
  
  @Override
  public void teleopInit() {
    lights.setLights(LightingSetting.TELEOP);
    IntakePivotSubsystem.getInstance().setBrake();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  
  @Override
  public void teleopPeriodic() {}
  
  @Override
  public void teleopExit() {}
  
  @Override
  public void testInit() {
    //IntakePivotSubsystem.getInstance().setBrake();
    //CommandScheduler.getInstance().cancelAll();
    //new IntakeSpeedControl().schedule();
    //new DefaultPivot().schedule();

    lights.setLights(LightingSetting.GAME);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**OI BRUV HAVE YOU GOT A LICENSE FOR THAT*/
  public void itsTeaTimeInit() {
    System.out.println("OI BRUV HAVE YOU GOT A LICENSE FOR THAT");
  }
}
