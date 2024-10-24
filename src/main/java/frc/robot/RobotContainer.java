// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AmpWaitScore;
import frc.robot.commands.DefaultPivot;
import frc.robot.commands.IntakeAdjustDown;
import frc.robot.commands.IntakeAdjustUp;
import frc.robot.commands.IntakeAutoPickup;
import frc.robot.commands.IntakeAutoRetract;
import frc.robot.commands.IntakeAutoRetractAuton;
import frc.robot.commands.IntakeFromShooter;
import frc.robot.commands.LimelightLEDsBlink;
import frc.robot.commands.LimelightLightingDefault;
import frc.robot.commands.PivotAdjustDown;
import frc.robot.commands.PivotAdjustUp;
import frc.robot.commands.PivotAmp;
import frc.robot.commands.PivotBase;
import frc.robot.commands.PivotClimb;
import frc.robot.commands.PivotFar;
import frc.robot.commands.PivotReset;
import frc.robot.commands.RumbleRumble;
import frc.robot.commands.SetClimberSpeed;
import frc.robot.commands.SetIntakePivotSpeed;
import frc.robot.commands.SetPosition;
import frc.robot.commands.SwerveResetGyro;
import frc.robot.commands.Intake.IntakeBasePos;
import frc.robot.commands.Intake.IntakeClosePos;
import frc.robot.commands.Intake.IntakeFarPos;
import frc.robot.commands.Intake.IntakeFloorPos;
import frc.robot.commands.Intake.IntakeFromSource;
import frc.robot.commands.Intake.IntakeSpeedControl;
import frc.robot.commands.Intake.IntakeSpinIn;
import frc.robot.commands.Intake.IntakeSpinOut;
import frc.robot.commands.Intake.IntakeSpinStop;
import frc.robot.commands.Intake.IntakeWaitNote;
import frc.robot.commands.Intake.IntakeWaitPosition;
import frc.robot.commands.Shooter.AutoAimIntake;
import frc.robot.commands.Shooter.AutoAimShooter;
import frc.robot.commands.Shooter.ShooterAmpLoad;
import frc.robot.commands.Shooter.ShooterAmpScore;
import frc.robot.commands.Shooter.ShooterAutoShoot;
import frc.robot.commands.Shooter.ShooterAutoShootAuton;
import frc.robot.commands.Shooter.ShooterAutoShootStop;
import frc.robot.commands.Shooter.ShooterAutoShootTeleop;
import frc.robot.commands.Shooter.ShooterFarShoot;
import frc.robot.commands.Shooter.ShooterIntake;
import frc.robot.commands.Shooter.ShooterRevWait;
import frc.robot.commands.Shooter.ShooterShoot;
import frc.robot.commands.Shooter.ShooterShootSlow;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.commands.Shooter.ShooterWaitPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSpinSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  private enum Autos {
    Nothing, ShootOne, FivePieceWingCenterSub, ThreePieceSourceCenterSub, FourPieceAmpCenterSub, FourPieceAmpCenter,
    FivePieceWingCenter, MessUp, ShootAndMove
  }

  // private double MaxSpeed = 5; // 6 meters per second desired top speed
  // private double MaxSpeed = 7;
  private double MaxSpeed = 5.5;
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // private final CommandXboxController joystick = new CommandXboxController(0);
  // // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.075).withRotationalDeadband(MaxAngularRate * Constants.OperatorConstants.deadBandMove) // Add a 7.5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.075).withRotationalDeadband(MaxAngularRate * Constants.OperatorConstants.deadBandRot)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  // private final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new
  // SwerveRequest.PointWheelsAt();
  // private final Telemetry logger = new Telemetry(MaxSpeed);
  private SendableChooser<Autos> newautopick;
  private Command autoCommand;

  public RobotContainer() {
    configureBindings();
    swerve = SwerveSubsystem.getInstance();
    LimelightSubsystem.getInstance();
    IntakePivotSubsystem.getInstance().setDefaultCommand(new IntakeSpeedControl());
    PivotSubsystem.getInstance().setDefaultCommand(new DefaultPivot());
    ClimberSubsystem.getInstance().setDefaultCommand(new SetClimberSpeed(0));

    NamedCommands.registerCommand("IntakeAutoPickup", new IntakeAutoPickup());
    NamedCommands.registerCommand("IntakeWaitNote", new IntakeWaitNote());
    NamedCommands.registerCommand("IntakeWaitPosition", new IntakeWaitPosition());
    NamedCommands.registerCommand("IntakeAutoRetract", new IntakeAutoRetract());
    NamedCommands.registerCommand("ShooterFarShoot", new ShooterFarShoot());
    NamedCommands.registerCommand("ShooterShoot", new ShooterShoot());
    NamedCommands.registerCommand("ShooterShootSlow", new ShooterShootSlow());
    NamedCommands.registerCommand("ShooterRevWait", new ShooterRevWait());
    NamedCommands.registerCommand("ShooterAutoShoot", new ShooterAutoShoot());
    NamedCommands.registerCommand("ShooterAutoShootAuton", new ShooterAutoShootAuton());
    NamedCommands.registerCommand("ShooterAutoShootStop", new ShooterAutoShootStop());
    NamedCommands.registerCommand("IntakeFloorPos", new IntakeFloorPos());
    NamedCommands.registerCommand("Nothing", new SetPosition());
    newautopick = new SendableChooser<>();
    newautopick.addOption("Nothing", Autos.Nothing);
    newautopick.addOption("ShootOne", Autos.ShootOne); // shoots and waits
    newautopick.addOption("ShootAndMove", Autos.ShootAndMove);
    newautopick.addOption("MessUp", Autos.MessUp);
    newautopick.addOption("3PieceSourceCenterSub", Autos.ThreePieceSourceCenterSub);
    newautopick.addOption("4PieceAmpCenter", Autos.FourPieceAmpCenter);
    newautopick.addOption("4PieceAmpCenterSub", Autos.FourPieceAmpCenterSub);
    newautopick.addOption("5PieceWingCenter", Autos.FivePieceWingCenter);
    newautopick.addOption("5PieceWingCenterSub", Autos.FivePieceWingCenterSub);
    // SUPER ULTIMATE PATH!!!!!!! (WATCH OUT, LIBERALS)
    Shuffleboard.getTab("auto").add("auto", newautopick).withPosition(0, 0).withSize(3, 1);
    System.out.println("RC");
    LightingSubsystem.getInstance();
  }

  private void configureBindings() {
    LimelightSubsystem.getInstance().setDefaultCommand(new LimelightLightingDefault());
    new Trigger(() -> IntakeSpinSubsystem.getInstance().isIntake()).onTrue(new LimelightLEDsBlink().withTimeout(0.5));
    new Trigger(() -> ShooterSubsystem.getInstance().isAtRPS()).onTrue(new LimelightLEDsBlink().withTimeout(0.5));
    // new Trigger(()-> Timer.getMatchTime()<30).whileTrue(new
    // LEDCommand(LightingSetting.AUTO));
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityY(-Inputs.getTranslationY() * MaxSpeed) // Drive forward with
            // negative Y (forward)
            .withVelocityX(-Inputs.getTranslationX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(((Inputs.getRotation() * MaxAngularRate))) // Drive counterclockwise with negative X
                                                                           // (left)
        ));

    Inputs.getRobotCentric()
        .whileTrue(drivetrain.applyRequest(() -> robotDrive.withVelocityY(Inputs.getTranslationY() * MaxSpeed) // Drive
                                                                                                               // forward
                                                                                                               // with
            // negative Y (forward)
            .withVelocityX(Inputs.getTranslationX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(((Inputs.getRotation() * MaxAngularRate)))));

    Inputs.getResetGyro().onTrue(new SwerveResetGyro());
    Inputs.getAutoIntakeUp().onTrue(new IntakeAutoRetract());
    // Inputs.getAutoIntakeUp2().onTrue(new IntakeAutoRetract());
    Inputs.getAutoShootStop().onTrue(
        new ShooterAutoShootTeleop().andThen(new ParallelRaceGroup(new IntakeBasePos(), new ShooterWaitPosition())));
    Inputs.getAmpTransfer().onTrue(new SequentialCommandGroup(new IntakeClosePos().raceWith(new IntakeWaitPosition()),
        new ShooterAmpLoad(), new WaitCommand(0.2), new IntakeSpinOut(),  new WaitCommand(.5), new ShooterStop(), new IntakeSpinStop()));
    
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
    // Inputs.getShooterFarShoot().onTrue(new ShooterFarShoot());
    Inputs.getShooterFarShoot().onTrue(new ShooterFarShoot());
    //Inputs.getSourceIntake().whileTrue(new IntakeFromShooter().andThen(new ShooterStop()).andThen(new IntakeSpinStop()));
    Inputs.getSourceIntake().onTrue(new IntakeFromShooter());
    Inputs.getAutoIntakeDown().onTrue(new SequentialCommandGroup(new IntakeAutoPickup(), new IntakeFloorPos()));
    // Inputs.getAutoIntakeDown().onTrue(new SequentialCommandGroup(new IntakeAutoPickup(), new ParallelRaceGroup(new IntakeFloorPos(), new IntakeWaitNote()), new IntakeAutoRetract()));
    // Inputs.getAutoIntakeDown2().onTrue(new SequentialCommandGroup(new IntakeAutoPickup(), new IntakeWaitNote(), new IntakeAutoRetract()));

    // Inputs.getShooterAdjustUp().onTrue(new PivotAdjustUp());
    Inputs.getShooterStop2().onTrue(new ShooterStop());
    Inputs.getClimberMovingUp().whileTrue(new SetClimberSpeed(1));
    Inputs.getClimberMovingDown().whileTrue(new SetClimberSpeed(-1));
    Inputs.getIntakeFromShooter().whileTrue(new ShooterIntake());
    Inputs.getAmpAutoOuttake()
        .onTrue(new SequentialCommandGroup(new ShooterAmpLoad(), new IntakeSpinOut(),
            new ParallelRaceGroup(new PivotAmp(), new IntakeClosePos(),
                new SequentialCommandGroup(new WaitCommand(0.1), new ShooterWaitPosition(), new IntakeSpinStop(),
                    new ShooterAmpScore(),
                    new AmpWaitScore().withTimeout(2),
                    new ShooterStop())),
            new ParallelRaceGroup(new PivotBase(), new IntakeBasePos(),
                new SequentialCommandGroup(new WaitCommand(.1), new ShooterWaitPosition())).withTimeout(1.25))
            .andThen(new PivotReset()));

    Inputs.getPivotReset().onTrue(new PivotReset()); // Inputs.getAutoShootStop3().onTrue(new
                                                     // ShooterAutoShootTeleop().andThen(new ParallelRaceGroup(new
                                                     // IntakeBasePos(), new WaitCommand(0.5))));

    Inputs.getShooterAdjustDown().onTrue(new PivotAdjustDown());
    Inputs.getShooterAdjustUp().onTrue(new PivotAdjustUp());
    Inputs.getIntakeAdjustDown().onTrue(new IntakeAdjustDown());
    Inputs.getIntakeAdjustUp().onTrue(new IntakeAdjustUp());
    // Inputs.getAutoShootStop2().onTrue(new ShooterAutoShootTeleop().andThen(new
    // ParallelRaceGroup(new IntakeBasePos(), new WaitCommand(0.5))));
    // //getAutoAlignShooter
    Inputs.getAutoAlignShooter().onTrue(new SequentialCommandGroup(new AutoAimIntake(), new ShooterWaitPosition()));
    // Inputs.getTurnToSpeaker().onTrue(swerve.faceSpeaker());
    Inputs.getAutoAimShooter().whileTrue(new AutoAimShooter());

    Inputs.getPivotClimbPosition().whileTrue(new PivotClimb());
    Inputs.getPivotMoveDown().whileTrue(new SetIntakePivotSpeed(-Constants.PivotConstants.smallAdjustment));
    Inputs.getPivotMoveUp().whileTrue(new SetIntakePivotSpeed(Constants.PivotConstants.smallAdjustment));
    // Inputs. 0 goToAmp().onTrue(swerve.moveToAmp());
    Inputs.getToggleFaceSpeaker()
        .whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(Inputs.getTranslationY() * MaxSpeed) // Drive
                                                                                                          // forward
                                                                                                          // with
            // negative Y (forward)
            .withVelocityX(Inputs.getTranslationX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(swerve.getTargetSlope(
                swerve.getPose().getX(),
                swerve.getPose().getY(),
                swerve.getSpeakerX(),
                swerve.getSpeakerY()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    //Inputs.getAmpTransfer().whileTrue(new IntakeFromSource());

  }

  public void generateAndLoad() {
    autoCommand = generateAutoCommand();
  }

  public void generateTrajectories(String name){
    List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("5PieceWingCenterSub");
    for(PathPlannerPath path:paths){
      autoTraj.add(TrajectoryGenerator.generateTrajectory(path.getPathPoses(), new TrajectoryConfig(MaxSpeed, MaxAngularRate)));
      // autoTraj.add(path.getPathPoses());   
    }
  }

  // List<List<Pose2d>> autoTraj = new List<List<Pose2d>>();
  ArrayList<Trajectory> autoTraj = new ArrayList<>();

  private Command generateAutoCommand(){
    switch(newautopick.getSelected()){
      case ShootOne:
        return new SequentialCommandGroup(new ShooterAutoShootTeleop(), new WaitCommand(50000));
      case FivePieceWingCenterSub:
        generateTrajectories("5PieceWingCenterSub");
        return SwerveSubsystem.getInstance().getAuto("5PieceWingCenterSub");
      case ThreePieceSourceCenterSub:
        generateTrajectories("3PieceSourceCenterSub");
        return SwerveSubsystem.getInstance().getAuto("3PieceSourceCenterSub");
      case FourPieceAmpCenterSub:
        generateTrajectories("4PieceAmpCenterSub");
        return SwerveSubsystem.getInstance().getAuto("4PieceAmpCenterSub");
      case FourPieceAmpCenter:
        generateTrajectories("4PieceAmpCenter");
        return SwerveSubsystem.getInstance().getAuto("4PieceAmpCenter");
      case FivePieceWingCenter:
        generateTrajectories("5PieceWingCenter");
        return SwerveSubsystem.getInstance().getAuto("5PieceWingCenter");
      case MessUp:
        generateTrajectories("MessUp");
        return SwerveSubsystem.getInstance().getAuto("MessUp");
      case ShootAndMove:
        generateTrajectories("1PieceSourceMove");
        return SwerveSubsystem.getInstance().getAuto("1PieceSourceMove");
      // case OtherAuto:
      //   return new AutoFollowPath("KajillionNote.1", "KajillionNote.2", "KajillionNote.3", "KajillionNote.4", "KajillionNote.5", "KajillionNote.6", "KajillionNote.7");
      default: //Autos.Nothing
        return new WaitCommand(50000);
    }
  }

  public ArrayList<Trajectory> getAutoTrajectory() {
    return autoTraj;
    // return null;
  }

  public Command getAutonomousCommand() {
    if (autoCommand == null) {
      autoCommand = generateAutoCommand();
    }
    return autoCommand;
  }
}

// STOP!
// This is the end of the robot container section of the SAT. Please do not move
// on until prompted to do so.