// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShooterAmpLoad;
import frc.robot.commands.ShooterAmpScore;
import frc.robot.commands.ShooterShoot;
import frc.robot.commands.ShooterStop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.commands.Intake.IntakeBasePos;
import frc.robot.commands.Intake.IntakeFloorPos;
import frc.robot.commands.Intake.IntakeSpeedControl;
import frc.robot.commands.Intake.IntakeSpinIn;
import frc.robot.commands.Intake.IntakeSpinOut;
import frc.robot.commands.Intake.IntakeSpinStop;
import frc.robot.commands.DefaultPivot;
import frc.robot.commands.PivotAmp;
import frc.robot.commands.PivotBase;
import frc.robot.commands.PivotFar;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  public RobotContainer() {
    configureBindings();
    System.out.println("RC");
  }

  private void configureBindings() {
    Inputs.getShooterShoot().onTrue(new ShooterShoot());
    Inputs.getShooterStop().onTrue(new ShooterStop());
    Inputs.getShooterAmpLoad().onTrue(new ShooterAmpLoad());
    Inputs.getShooterAmpScore().onTrue(new ShooterAmpScore());
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(Inputs.getTranslationX() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(Inputs.getTranslationY() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(Inputs.getRotation() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    Inputs.getResetGyro().onTrue()
    Inputs.getIntakeSpinIn().onTrue(new IntakeSpinIn());
    Inputs.getIntakeBasePos().onTrue(new IntakeBasePos());
    Inputs.getIntakeFloorPos().onTrue(new IntakeFloorPos());
    Inputs.getIntakeSpinOut().onTrue(new IntakeSpinOut());
    Inputs.getIntakeSpinStop().onTrue(new IntakeSpinStop());
    Inputs.getPivotAmp().onTrue(new PivotAmp());
    Inputs.getPivotBase().onTrue(new PivotBase());
    Inputs.getPivotFar().onTrue(new PivotFar());
    IntakePivotSubsystem.getInstance().setDefaultCommand(new IntakeSpeedControl());
    PivotSubsystem.getInstance().setDefaultCommand(new DefaultPivot());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
