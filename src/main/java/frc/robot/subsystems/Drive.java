// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private CANSparkMax rightMotor1;
  private CANSparkMax rightMotor2;
  private CANSparkMax leftMotor1;
  private CANSparkMax leftMotor2;
  private AbsoluteEncoder rightAbsoluteEncoder;
  private AbsoluteEncoder leftAbsoluteEncoder;
  private PIDController driveController;
  private SimpleMotorFeedforward ff;

  
  public Drive() {
    PIDController driveController = new PIDController(0, 0, 0);

    rightAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI * 1.0);
    leftAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI * 1);

    ff = new SimpleMotorFeedforward(0, 0);

    leftMotor2.follow(leftMotor1, false);
    rightMotor2.follow(rightMotor1, false);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void setMotorSpeed(double speed)
  {
    leftMotor1.set(speed);
    leftMotor2.set(speed);
    rightMotor1.set(speed);
    rightMotor2.set(speed);
  }

  public void setTargetDistance(double targetDistance)
  {
    driveController.setSetpoint(targetDistance);
  }

  public void setDrivePosition(double distance)
  {
    driveController.setSetpoint(distance);
  }

  public void moveForward()
  {
    leftMotor1.set(1);
    leftMotor2.set(1);
    rightMotor1.set(1);
    rightMotor2.set(1);
  }

  public void moveBackward()
  {
    leftMotor1.set(-1);
    leftMotor2.set(-1);
    rightMotor1.set(-1);
    rightMotor2.set(-1);
  }

  public void stop()
  {
    leftMotor1.set(0);
    leftMotor2.set(0);
    rightMotor1.set(0);
    rightMotor2.set(0);
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    leftMotor1.setVoltage(driveController.calculate(leftAbsoluteEncoder.getPosition()) + ff.calculate(0, 0, 0));
    rightMotor1.setVoltage(driveController.calculate(rightAbsoluteEncoder.getPosition()) + ff.calculate(0, 0, 0));

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
