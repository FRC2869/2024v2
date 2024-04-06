// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.LEDs;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.Constants.ShooterConstants.LightingSetting;
// import frc.robot.subsystems.LightingSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class LEDCommand extends InstantCommand {
//   private LightingSubsystem leds;
//   private LightingSetting mode;

//   public LEDCommand(LightingSetting ledMode) {
//     leds = LightingSubsystem.getInstance();
//     mode = ledMode;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     leds.setLights(mode);
//   }
// }
