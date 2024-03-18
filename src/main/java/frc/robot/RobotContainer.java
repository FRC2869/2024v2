// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DefaultPivot;
import frc.robot.commands.DrivetrainResetGyro;
import frc.robot.commands.IntakeAutoPickup;
import frc.robot.commands.IntakeAutoRetract;
import frc.robot.commands.PivotAmp;
import frc.robot.commands.PivotBase;
import frc.robot.commands.PivotFar;
import frc.robot.commands.Intake.IntakeBasePos;
import frc.robot.commands.Intake.IntakeFarPos;
import frc.robot.commands.Intake.IntakeFloorPos;
import frc.robot.commands.Intake.IntakeSpeedControl;
import frc.robot.commands.Intake.IntakeSpinIn;
import frc.robot.commands.Intake.IntakeSpinOut;
import frc.robot.commands.Intake.IntakeSpinStop;
import frc.robot.commands.Intake.IntakeWaitNote;
import frc.robot.commands.Shooter.AimAtSpeaker;
import frc.robot.commands.Shooter.ShooterAmpLoad;
import frc.robot.commands.Shooter.ShooterAmpScore;
import frc.robot.commands.Shooter.ShooterAutoShoot;
import frc.robot.commands.Shooter.ShooterAutoShootStop;
import frc.robot.commands.Shooter.ShooterAutoShootTeleop;
import frc.robot.commands.Shooter.ShooterFarShoot;
import frc.robot.commands.Shooter.ShooterRevWait;
import frc.robot.commands.Shooter.ShooterShoot;
import frc.robot.commands.Shooter.ShooterShootSlow;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private Swerve swerve;
  private enum Autos {
		Nothing, Forward, ShootOne, ShootPickup, SubWooferAuto, BasicPath, SubWooferAutoR, SubWooferAutoR2
	}
  private double MaxSpeed = 5; // 6 meters per second desired top speed
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // private final Telemetry logger = new Telemetry(MaxSpeed);
  private SendableChooser<Autos> newautopick;

  public RobotContainer() {
    configureBindings();
    swerve = Swerve.getInstance();
    IntakePivotSubsystem.getInstance().setDefaultCommand(new IntakeSpeedControl());
    PivotSubsystem.getInstance().setDefaultCommand(new DefaultPivot());
    // Map<String, Command> eventMap = new HashMap<>();
    // eventMap.put("IntakeAutoPickup", new IntakeAutoPickup());
    // eventMap.put("IntakeWaitNote", new IntakeWaitNote());
    // eventMap.put("IntakeAutoRetract", new IntakeAutoRetract());
    // eventMap.put("ShooterFarShoot", new ShooterFarShoot());
    // eventMap.put("ShooterAutoShoot", new ShooterAutoShoot());
    // NamedCommands.registerCommands(eventMap);
    NamedCommands.registerCommand("IntakeAutoPickup", new IntakeAutoPickup());
    NamedCommands.registerCommand("IntakeWaitNote", new IntakeWaitNote());
    NamedCommands.registerCommand("IntakeAutoRetract", new IntakeAutoRetract());
    NamedCommands.registerCommand("ShooterFarShoot", new ShooterFarShoot());
    NamedCommands.registerCommand("ShooterShoot", new ShooterShoot());
    NamedCommands.registerCommand("ShooterShootSlow", new ShooterShootSlow());
    NamedCommands.registerCommand("ShooterRevWait", new ShooterRevWait());
    NamedCommands.registerCommand("ShooterAutoShoot", new ShooterAutoShoot());
    NamedCommands.registerCommand("ShooterAutoShootStop", new ShooterAutoShootStop());
    NamedCommands.registerCommand("PPath1", swerve.getPathPlannerTrajectory("PPath1"));
    NamedCommands.registerCommand("PPath2", swerve.getPathPlannerTrajectory("PPath2"));
    NamedCommands.registerCommand("PPath3", swerve.getPathPlannerTrajectory("PPath3"));
    NamedCommands.registerCommand("Rev", new ShooterRevWait());
    NamedCommands.registerCommand("Nothing", new WaitCommand(0));
    newautopick = new SendableChooser<>();
		newautopick.addOption("Nothing", Autos.Nothing);
		newautopick.addOption("Forward", Autos.Forward);	//2m path
		newautopick.addOption("ShootOne", Autos.ShootOne);	//shoots and waits
		newautopick.addOption("ShootPickup", Autos.ShootPickup);	
		newautopick.addOption("SubWooferAuto", Autos.SubWooferAuto);	
		newautopick.addOption("BasicPath", Autos.BasicPath);	
		newautopick.addOption("SubWooferAutoR", Autos.SubWooferAutoR);
		newautopick.addOption("SubWooferAutoR2", Autos.SubWooferAutoR2);
    //SUPER ULTIMATE PATH!!!!!!! (WATCH OUT, LIBERALS)
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
    Inputs.getIntakeSpinOut().onTrue(new IntakeSpinOut());
    Inputs.getIntakeSpinStop().onTrue(new IntakeSpinStop());
    Inputs.getPivotAmp().whileTrue(new PivotAmp());
    Inputs.getPivotBase().whileTrue(new PivotBase());
    
    //Operator board
    // Inputs.getAutoIntakeDown().onTrue(new IntakeAutoPickup());
    Inputs.getAutoIntakeUp().onTrue(new IntakeAutoRetract());
    Inputs.getAutoShootStop().onTrue(new ShooterAutoShootTeleop().andThen(new IntakeBasePos()));
    Inputs.getAmpTransfer().onTrue(new SequentialCommandGroup(new IntakeSpinOut(), new ShooterAmpLoad()));
    Inputs.getIntakeFloorPos().whileTrue(new IntakeFloorPos());

    Inputs.getIntakeBasePos().whileTrue(new IntakeBasePos());
    Inputs.getIntakeFar().onTrue(new IntakeFarPos());
    Inputs.getIntakeSpinOut2().onTrue(new IntakeSpinOut());
    Inputs.getIntakeSpinStop2().onTrue(new IntakeSpinStop());
    Inputs.getIntakeSpinIn2().onTrue(new IntakeSpinIn());

    Inputs.getPivotFar().onTrue(new PivotFar());
    Inputs.getShooterShoot2().onTrue(new ShooterShoot());
    Inputs.getPivotBase2().whileTrue(new PivotBase());
    Inputs.getPivotAmp2().whileTrue(new PivotAmp());
    Inputs.getShooterFarShoot().onTrue(new ShooterFarShoot());
    
    Inputs.getAimAtSpeaker().onTrue(new AimAtSpeaker());

    // LightingSubsystem.getInstance().setDefaultCommand(new LEDCommand(LightingSetting.CANSHOOT));
    Inputs.getAutoIntakeDown().onTrue(new SequentialCommandGroup(new IntakeAutoPickup(), new IntakeWaitNote(), new IntakeAutoRetract()));
    
    // Inputs.getLeft().onTrue(new Move(-1));
    // Inputs.getRight().onTrue(new SMove(1));
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
        return Swerve.getInstance().getAuto("TwoPieceMiddle");
      case SubWooferAutoR:
        return Swerve.getInstance().getAuto("OnePiece(not)Mid (The One Piece is real)");
      case SubWooferAutoR2:
        return Swerve.getInstance().getAuto("5PieceWingCenter");
      default: //Autos.Nothing
        return new WaitCommand(50000);
    }
  }
}

//Stop right there!  If you want to continue reading, a donation of, like, aa bajillion dollars is REQUIRED!  Thank you.