// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.Elevator;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// /** An example command that uses an example subsystem. */
// public class ElevatorCommand extends CommandBase {
//   private final Elevator elevatorSubsystem;
//   private final double speed;
//   /**
//    * Creates a new ExampleCommand.
//    *
//    */
//   public ElevatorCommand(Elevator elevatorSubsystem, double speed) {
//     this.elevatorSubsystem = elevatorSubsystem;
//     this.speed = speed;
//     addRequirements(elevatorSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     elevatorSubsystem.setMotorSpeed(speed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     elevatorSubsystem.setMotorSpeed(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
