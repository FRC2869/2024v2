// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultPivot;
import frc.robot.commands.IntakeAutoPickup;
import frc.robot.commands.IntakeAutoRetract;
import frc.robot.commands.LimelightLEDsBlink;
import frc.robot.commands.LimelightLightingDefault;
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
import frc.robot.commands.Intake.IntakeWaitPosition;
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
import frc.robot.commands.Shooter.ShooterWaitPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSpinSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private enum Autos {
		Nothing, ShootOne, FivePieceWingCenterSub, ThreePieceSourceCenterSub, FourPieceAmpCenterSub, FourPieceAmpCenter, FivePieceWingCenter, MessUp
	}
  // private double MaxSpeed = 5; // 6 meters per second desired top speed
  // private double MaxSpeed = 7;
  private double MaxSpeed = 5.5;
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.075).withRotationalDeadband(MaxAngularRate * 0.075) // Add a 7.5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final RobotCentric robotDrive = new SwerveRequest.RobotCentric()
  .withDeadband(MaxSpeed * 0.075).withRotationalDeadband(MaxAngularRate * 0.075)
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
                                                               // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // private final Telemetry logger = new Telemetry(MaxSpeed);
  private SendableChooser<Autos> newautopick;
  private Command autoCommand;

  public RobotContainer() {
    configureBindings();
    swerve = SwerveSubsystem.getInstance();
    LimelightSubsystem.getInstance();
    IntakePivotSubsystem.getInstance().setDefaultCommand(new IntakeSpeedControl());
    PivotSubsystem.getInstance().setDefaultCommand(new DefaultPivot());

    NamedCommands.registerCommand("IntakeAutoPickup", new IntakeAutoPickup());
    NamedCommands.registerCommand("IntakeWaitNote", new IntakeWaitNote());
    NamedCommands.registerCommand("IntakeWaitPosition", new IntakeWaitPosition());
    NamedCommands.registerCommand("IntakeAutoRetract", new IntakeAutoRetract());
    NamedCommands.registerCommand("ShooterFarShoot", new ShooterFarShoot());
    NamedCommands.registerCommand("ShooterShoot", new ShooterShoot());
    NamedCommands.registerCommand("ShooterShootSlow", new ShooterShootSlow());
    NamedCommands.registerCommand("ShooterRevWait", new ShooterRevWait());
    NamedCommands.registerCommand("ShooterAutoShoot", new ShooterAutoShoot());
    NamedCommands.registerCommand("ShooterAutoShootStop", new ShooterAutoShootStop());
    NamedCommands.registerCommand("Nothing", new WaitCommand(0));
    newautopick = new SendableChooser<>();
		newautopick.addOption("Nothing", Autos.Nothing);
		newautopick.addOption("ShootOne", Autos.ShootOne);	//shoots and waits
		newautopick.addOption("MessUp", Autos.MessUp);
		newautopick.addOption("3PieceSourceCenterSub", Autos.ThreePieceSourceCenterSub);
    newautopick.addOption("4PieceAmpCenter", Autos.FourPieceAmpCenter);
		newautopick.addOption("4PieceAmpCenterSub", Autos.FourPieceAmpCenterSub);
    newautopick.addOption("5PieceWingCenter", Autos.FivePieceWingCenter);
		newautopick.addOption("5PieceWingCenterSub", Autos.FivePieceWingCenterSub);
    //SUPER ULTIMATE PATH!!!!!!! (WATCH OUT, LIBERALS)
		Shuffleboard.getTab("auto").add("auto", newautopick).withPosition(0, 0).withSize(3, 1);
    System.out.println("RC");
  }

  private void configureBindings() {
    LimelightSubsystem.getInstance().setDefaultCommand(new LimelightLightingDefault());
    new Trigger(()-> IntakeSpinSubsystem.getInstance().isIntake()).onTrue(new LimelightLEDsBlink().withTimeout(0.5));
    new Trigger(()-> ShooterSubsystem.getInstance().isAtRPS()).onTrue(new LimelightLEDsBlink().withTimeout(0.5));
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityY(Inputs.getTranslationY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityX(Inputs.getTranslationX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate((( Inputs.getRotation() * MaxAngularRate))) // Drive counterclockwise with negative X (left)
        ));

    Inputs.getRobotCentric().whileTrue(drivetrain.applyRequest(() -> robotDrive.withVelocityY(Inputs.getTranslationY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityX(Inputs.getTranslationX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate((( Inputs.getRotation() * MaxAngularRate)))));
            
    Inputs.getResetGyro().onTrue(new SwerveResetGyro());
    Inputs.getAutoIntakeUp().onTrue(new IntakeAutoRetract());
    // Inputs.getAutoIntakeUp2().onTrue(new IntakeAutoRetract());
    Inputs.getAutoShootStop().onTrue(new ShooterAutoShootTeleop().andThen(new ParallelRaceGroup(new IntakeBasePos(), new ShooterWaitPosition())));
    Inputs.getAmpTransfer().onTrue(new SequentialCommandGroup(new IntakeBasePos().raceWith(new IntakeWaitPosition()), new IntakeSpinOut(), new ShooterAmpLoad(), new WaitCommand(.5), new ShooterStop(), new IntakeSpinStop()));
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
    Inputs.getShooterStop2().onTrue(new ShooterStop());
    Inputs.getAmpAutoOuttake().onTrue(new SequentialCommandGroup(new ShooterAmpLoad(), new ParallelRaceGroup(new PivotAmp(), new SequentialCommandGroup(new WaitCommand(0.1), new ShooterWaitPosition())), 
                                                                  new ShooterAmpScore(), 
                                                                  new ShooterRevWait().withTimeout(1), 
                                                                  new ShooterStop(), 
                                                                  new ParallelRaceGroup(new PivotBase(), new ShooterWaitPosition())));
    // Inputs.getAutoShootStop3().onTrue(new ShooterAutoShootTeleop().andThen(new ParallelRaceGroup(new IntakeBasePos(), new WaitCommand(0.5))));
    // Inputs.getAutoShootStop2().onTrue(new ShooterAutoShootTeleop().andThen(new ParallelRaceGroup(new IntakeBasePos(), new WaitCommand(0.5))));
    // //getAutoAlignShooter
    Inputs.getAutoAlignShooter().onTrue(new AimAtSpeaker());
    // Inputs.getTurnToSpeaker().onTrue(swerve.faceSpeaker());
    // Inputs.  0  goToAmp().onTrue(swerve.moveToAmp());
    Inputs.getToggleFaceSpeaker().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(Inputs.getTranslationY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityX(Inputs.getTranslationX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(swerve.getTargetSlope(
              swerve.getPose().getX(),
              swerve.getPose().getY(),
              swerve.getSpeakerX(),
              swerve.getSpeakerY()
            )) // Drive counterclockwise with negative X (left)
        ));

    
  }

  

   public void generateAndLoad(){
     autoCommand = generateAutoCommand();
   }
   List<List<Pose2d>> autoTraj = null;
  private Command generateAutoCommand(){
    List<PathPlannerPath> paths;
    switch(newautopick.getSelected()){
      case ShootOne:
        return new SequentialCommandGroup(new ShooterAutoShootTeleop(), new WaitCommand(50000));
      case FivePieceWingCenterSub:
        paths = PathPlannerAuto.getPathGroupFromAutoFile("5PieceWingCenterSub");
        for(PathPlannerPath path:paths){
          autoTraj.add(path.getPathPoses());
        }
        return SwerveSubsystem.getInstance().getAuto("5PieceWingCenterSub");
      case ThreePieceSourceCenterSub:
        paths = PathPlannerAuto.getPathGroupFromAutoFile("3PieceSourceCenterSub");
        for(PathPlannerPath path:paths){
          autoTraj.add(path.getPathPoses());
        }
        return SwerveSubsystem.getInstance().getAuto("3PieceSourceCenterSub");
      case FourPieceAmpCenterSub:
        paths = PathPlannerAuto.getPathGroupFromAutoFile("4PieceAmpCenterSub");
        for(PathPlannerPath path:paths){
          autoTraj.add(path.getPathPoses());
        }
        return SwerveSubsystem.getInstance().getAuto("4PieceAmpCenterSub");
      case FourPieceAmpCenter:
        paths = PathPlannerAuto.getPathGroupFromAutoFile("4PieceAmpCenter");
        for(PathPlannerPath path:paths){
          autoTraj.add(path.getPathPoses());
        }
        return SwerveSubsystem.getInstance().getAuto("4PieceAmpCenter");
      case FivePieceWingCenter:
        paths = PathPlannerAuto.getPathGroupFromAutoFile("5PieceWingCenter");
        for(PathPlannerPath path:paths){
          autoTraj.add(path.getPathPoses());
        }
        return SwerveSubsystem.getInstance().getAuto("5PieceWingCenter");
      case MessUp:
        paths = PathPlannerAuto.getPathGroupFromAutoFile("MessUp");
        for(PathPlannerPath path:paths){
          autoTraj.add(path.getPathPoses());
        }
        return SwerveSubsystem.getInstance().getAuto("MessUp");
      // case OtherAuto:
      //   return new AutoFollowPath("KajillionNote.1", "KajillionNote.2", "KajillionNote.3", "KajillionNote.4", "KajillionNote.5", "KajillionNote.6", "KajillionNote.7");
      default: //Autos.Nothing
        return new WaitCommand(50000);
    }
  }

  public List<List<Pose2d>> getAutoTrajectory(){
    return autoTraj;
  }

  public Command getAutonomousCommand() {
    if(autoCommand==null){
      autoCommand = generateAutoCommand();
    }
    return autoCommand;
  }
}

//STOP!
//This is the end of the robot container section of the SAT.  Please do not move on until prompted to do so.