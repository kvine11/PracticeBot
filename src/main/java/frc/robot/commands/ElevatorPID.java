// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.Elevator;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// /** An example command that uses an example subsystem. */
// public class ElevatorPID extends CommandBase {
//     private final Elevator elevatorSubsystem;
//     private final PIDController pidController;
//   /**
//    * Creates a new ExampleCommand.
//    *
//    */
//   public ElevatorPID(Elevator elevatorSubsystem, double setpoint) {
//     this.elevatorSubsystem = elevatorSubsystem;
//     this.pidController = new PIDController(3, setpoint, 0.8);
//     pidController.setSetpoint(setpoint);
//     addRequirements(elevatorSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     pidController.reset();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double speed = pidController.calculate(elevatorSubsystem.getEncoderMeters());
//     elevatorSubsystem.setMotorSpeed(speed);
//       }

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
