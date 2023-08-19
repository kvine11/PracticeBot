// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class armPID extends CommandBase {
    private final Arm armSubsystem;
    private final PIDController pidController;
  /**
   * Creates a new ExampleCommand.
   *
   */
  public armPID(Arm armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    this.pidController = new PIDController(3, setpoint, 0.8);
    pidController.setSetpoint(setpoint);
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(armSubsystem.getEncoderMeters());
    armSubsystem.setMotorSpeed(speed);
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
