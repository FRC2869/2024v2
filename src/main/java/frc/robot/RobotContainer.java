// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SwerveResetGyro;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.Intake.IntakeBasePos;
import frc.robot.commands.Intake.IntakeFarPos;
import frc.robot.commands.Intake.IntakeFloorPos;
import frc.robot.commands.Intake.IntakeSpeedControl;
import frc.robot.commands.Intake.IntakeSpinIn;
import frc.robot.commands.Intake.IntakeSpinOut;
import frc.robot.commands.Intake.IntakeSpinStop;
import frc.robot.commands.Shooter.ShooterAmpLoad;
import frc.robot.commands.Shooter.ShooterAmpScore;
import frc.robot.commands.Shooter.ShooterAutoShoot;
import frc.robot.commands.Shooter.ShooterFarShoot;
import frc.robot.commands.Shooter.ShooterShoot;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.commands.DefaultPivot;
import frc.robot.commands.DrivetrainResetGyro;
import frc.robot.commands.IntakeAutoPickup;
import frc.robot.commands.IntakeAutoRetract;
import frc.robot.commands.PivotAmp;
import frc.robot.commands.PivotBase;
import frc.robot.commands.PivotFar;

public class RobotContainer {
  private enum Autos {
		Nothing, Forward, ShootOne, ShootPickup, SubWooferAuto, BasicPath
	}
  private double MaxSpeed = 8; // 6 meters per second desired top speed
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private SendableChooser<Autos> newautopick;

  public RobotContainer() {
    configureBindings();
    newautopick = new SendableChooser<>();
		newautopick.addOption("Nothing", Autos.Nothing);
		newautopick.addOption("Forward", Autos.Forward);	//2m path
		newautopick.addOption("ShootOne", Autos.ShootOne);	//shoots and waits
		newautopick.addOption("ShootPickup", Autos.ShootPickup);	
		newautopick.addOption("SubWooferAuto", Autos.SubWooferAuto);	
		newautopick.addOption("BasicPath", Autos.BasicPath);	
		Shuffleboard.getTab("auto").add("auto", newautopick).withPosition(0, 0).withSize(3, 1);
    System.out.println("RC");
  }

  private void configureBindings() {
    Inputs.getShooterShoot().onTrue(new ShooterShoot());
    Inputs.getShooterStop().onTrue(new ShooterStop());
    Inputs.getShooterAmpLoad().onTrue(new ShooterAmpLoad());
    Inputs.getShooterAmpScore().onTrue(new ShooterAmpScore());
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityY(Inputs.getTranslationY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityX(Inputs.getTranslationX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(Inputs.getRotation() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    Inputs.getResetGyro().onTrue(new DrivetrainResetGyro());
    Inputs.getIntakeSpinIn().onTrue(new IntakeSpinIn());
    Inputs.getIntakeBasePos().whileTrue(new IntakeBasePos());
    Inputs.getIntakeFloorPos().whileTrue(new IntakeFloorPos());
    Inputs.getIntakeSpinOut().onTrue(new IntakeSpinOut());
    Inputs.getIntakeSpinStop().onTrue(new IntakeSpinStop());
    Inputs.getPivotAmp().whileTrue(new PivotAmp());
    Inputs.getPivotBase().whileTrue(new PivotBase());
    Inputs.getPivotFar().onTrue(new PivotFar());
    Inputs.getIntakeFar().onTrue(new IntakeFarPos());
    IntakePivotSubsystem.getInstance().setDefaultCommand(new IntakeSpeedControl());
    PivotSubsystem.getInstance().setDefaultCommand(new DefaultPivot());
    Inputs.getAutoIntakeDown().onTrue(new IntakeAutoPickup());
    Inputs.getAutoIntakeUp().onTrue(new IntakeAutoRetract());
    Inputs.getAutoShoot().onTrue(new ShooterAutoShoot());
  }

  public Command getAutonomousCommand() {
    switch(newautopick.getSelected()){
      case ShootPickup:
        return new SequentialCommandGroup(new ShooterAutoShoot(), Swerve.getInstance().getTrajectory("TwoPiece"), new IntakeAutoPickup(), new ShooterAutoShoot());
      case ShootOne:
        return new SequentialCommandGroup(new ShooterAutoShoot(), new WaitCommand(50000));
      case Forward:
        return Swerve.getInstance().getTrajectory("2m");
      case SubWooferAuto:
        return new SequentialCommandGroup(new ShooterAutoShoot(), new ParallelCommandGroup(Swerve.getInstance().getTrajectory("TopBlueAutoPath"), 
        new SequentialCommandGroup(new WaitCommand(1), new IntakeAutoPickup())));
      case BasicPath:
        return new SequentialCommandGroup(new ShooterAutoShoot(), new IntakeSpinIn(), new ParallelRaceGroup(new IntakeFloorPos(), new WaitCommand(.75)),
        new ParallelRaceGroup(Swerve.getInstance().getTrajectory("TwoPieceMiddle"), new IntakeFloorPos()), new IntakeAutoPickup(), new IntakeAutoRetract(), new ShooterFarShoot() );
      default: //Autos.Nothing
        return new WaitCommand(50000);
    }
  }
}
