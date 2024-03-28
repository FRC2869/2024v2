// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoFollowPath;
import frc.robot.commands.DefaultPivot;
import frc.robot.commands.FaceSpeaker;
import frc.robot.commands.IntakeAutoPickup;
import frc.robot.commands.IntakeAutoRetract;
import frc.robot.commands.PivotAdjustUp;
import frc.robot.commands.PivotAmp;
import frc.robot.commands.PivotBase;
import frc.robot.commands.PivotFar;
import frc.robot.commands.SwerveResetGyro;
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
import frc.robot.commands.Shooter.ShooterAutoShootTeleop;
import frc.robot.commands.Shooter.ShooterFarShoot;
import frc.robot.commands.Shooter.ShooterShoot;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private enum Autos {
		Nothing, Forward, ShootOne, ShootPickup, SubWooferAuto, BasicPath, SubWooferAutoR, SubWooferAutoR2, SideAuto, OtherAuto, BestAuto
	}
  // private double MaxSpeed = 5; // 6 meters per second desired top speed
  // private double MaxSpeed = 7;
  private double MaxSpeed = 3;
  private double MaxAngularRate = 1 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.075).withRotationalDeadband(MaxAngularRate * 0.075) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final RobotCentric robotDrive = new SwerveRequest.RobotCentric()
  .withDeadband(MaxSpeed * 0.075).withRotationalDeadband(MaxAngularRate * 0.075)
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
                                                               // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // private final Telemetry logger = new Telemetry(MaxSpeed);
  private SendableChooser<Autos> newautopick;

  public RobotContainer() {
    configureBindings();
    swerve = SwerveSubsystem.getInstance();
    IntakePivotSubsystem.getInstance().setDefaultCommand(new IntakeSpeedControl());
    PivotSubsystem.getInstance().setDefaultCommand(new DefaultPivot());
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityY(Inputs.getTranslationY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityX(Inputs.getTranslationX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate((( Inputs.getRotation() * MaxAngularRate ) * SwerveSubsystem.getOverrideState()) + (swerve.getTargetSlope(
              swerve.getPose().getX(),
              swerve.getPose().getY(),
              swerve.getSpeakerX(),
              swerve.getSpeakerY()
            ) * Math.abs(SwerveSubsystem.getOverrideState() - 1))) // Drive counterclockwise with negative X (left)
        ));
    Inputs.getResetGyro().onTrue(new SwerveResetGyro());
    Inputs.getAutoIntakeUp().onTrue(new IntakeAutoRetract());
    // Inputs.getAutoIntakeUp2().onTrue(new IntakeAutoRetract());
    Inputs.getAutoShootStop().onTrue(new ShooterAutoShootTeleop().andThen(new ParallelRaceGroup(new IntakeBasePos(), new WaitCommand(0.5))));
    Inputs.getAmpTransfer().onTrue(new SequentialCommandGroup(new IntakeSpinOut(), new ShooterAmpLoad(), new WaitCommand(.5), new ShooterStop(), new IntakeSpinStop()));
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
    Inputs.getAutoIntakeDown().onTrue(new SequentialCommandGroup(new IntakeAutoPickup(), new IntakeWaitNote(), new IntakeAutoRetract()));
    // Inputs.getAutoIntakeDown2().onTrue(new SequentialCommandGroup(new IntakeAutoPickup(), new IntakeWaitNote(), new IntakeAutoRetract()));

    // Inputs.getShooterAdjustUp().onTrue(new PivotAdjustUp());
    // Inputs.getShooterStop2().onTrue(new ShooterStop());
    // Inputs.getAmpAutoOuttake().onTrue(new SequentialCommandGroup(new ParallelRaceGroup(new PivotAmp(), new WaitCommand(1)), new ShooterAmpScore(), new WaitCommand(.5), new ShooterStop(), new ParallelRaceGroup(new PivotBase(), new WaitCommand(.5))));
    // Inputs.getAutoShootStop3().onTrue(new ShooterAutoShootTeleop().andThen(new ParallelRaceGroup(new IntakeBasePos(), new WaitCommand(0.5))));
    // Inputs.getAutoShootStop2().onTrue(new ShooterAutoShootTeleop().andThen(new ParallelRaceGroup(new IntakeBasePos(), new WaitCommand(0.5))));
    // //getAutoAlignShooter
    // Inputs.getAutoAlignShooter().onTrue(new AimAtSpeaker());
    // Inputs.getTurnToSpeaker().onTrue(swerve.faceSpeaker());
    // Inputs.goToAmp().onTrue(swerve.moveToAmp());
    Inputs.getToggleFaceSpeaker().onTrue(new FaceSpeaker());

    
  }

  

  //  public void generateAndLoad(){
  //    autoCommand = generateAutoCommand();
  //  }

  private Command generateAutoCommand(){
    switch(newautopick.getSelected()){
      case ShootPickup:
        return new SequentialCommandGroup(new ShooterAutoShoot(), SwerveSubsystem.getInstance().getTrajectory("TwoPiece"), new IntakeAutoPickup(), new ShooterAutoShoot());
      case ShootOne:
        return new SequentialCommandGroup(new ShooterAutoShootTeleop(), new WaitCommand(50000));
      case Forward:
        return SwerveSubsystem.getInstance().getTrajectory("2m");
      case SubWooferAuto:
        return new SequentialCommandGroup(new ShooterAutoShoot(), new ParallelCommandGroup(SwerveSubsystem.getInstance().getTrajectory("TopBlueAutoPath"), 
        new SequentialCommandGroup(new WaitCommand(1), new IntakeAutoPickup())));
      case BasicPath:
        return SwerveSubsystem.getInstance().getAuto("TwoPieceMiddle");
      case SubWooferAutoR:
        return SwerveSubsystem.getInstance().getAuto("OnePiece(not)Mid (The One Piece is real)");
      case SubWooferAutoR2:
        return SwerveSubsystem.getInstance().getAuto("5PieceWingCenter");//5PieceWingCenter
      case SideAuto:
        return SwerveSubsystem.getInstance().getAuto("2PieceCenterFar");
      case BestAuto:
        return SwerveSubsystem.getInstance().getAuto("2PieceCenterFar");
      case OtherAuto:
        return new AutoFollowPath("KajillionNote.1", "KajillionNote.2", "KajillionNote.3", "KajillionNote.4", "KajillionNote.5", "KajillionNote.6", "KajillionNote.7");
      default: //Autos.Nothing
        return new WaitCommand(50000);
    }
  }

  public Command getAutonomousCommand() {
    // if(autoCommand==null){
    //   autoCommand = generateAutoCommand();
    // }
    return null;
  }
}

//STOP!
//This is the end of the robot container section of the SAT.  Please do not move on until prompted to do so.