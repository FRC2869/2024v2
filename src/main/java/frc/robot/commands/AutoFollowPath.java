// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shooter.ShooterAutoShoot;
import frc.robot.commands.Shooter.ShooterRevWait;
import frc.robot.commands.Shooter.ShooterShoot;
import frc.robot.commands.Shooter.ShooterShootSlow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFollowPath extends SequentialCommandGroup {
  /** Creates a new AutoFollowPath. */
  public AutoFollowPath(String... paths) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    for (int i = 0; i < paths.length; i++) {
      if (i % 2 == 0)
        addCommands(
          new ShooterShootSlow(),
          new ShooterRevWait(),
          new ShooterAutoShoot(),
          new ParallelCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(paths[i])),
            new SequentialCommandGroup(new WaitCommand(.25), new IntakeAutoPickup())
          )
        );
      else
        addCommands(
          new ParallelCommandGroup(
            new IntakeAutoRetract(),
            new ShooterShoot(),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(paths[i]))
          )
        );
    }
  }
}
