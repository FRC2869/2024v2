// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Intake.IntakeSpeedControl;
import frc.robot.subsystems.IntakePivotSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // System.out.println("INIT");
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    IntakePivotSubsystem.getInstance().setBrake();
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
    IntakePivotSubsystem.getInstance().setBrake();
    CommandScheduler.getInstance().cancelAll();
    new IntakeSpeedControl().schedule();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
