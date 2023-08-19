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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;
    private AbsoluteEncoder intakeEncoder;
    private PIDController intakeController;
    private SimpleMotorFeedforward ff;
    private final Encoder encoder = new Encoder(4, 5);
    private final double encoderConversion = 1 / 4096 * 0.1 * Math.PI;

  
  public Intake() 
  {
    PIDController intakeController = new PIDController(0, 0, 0);
    intakeEncoder.setPositionConversionFactor(2 * Math.PI * 1.0);
    ff = new SimpleMotorFeedforward(0, 0);
  

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
    intakeMotor.set(speed);
  }

  public void setTargetDistance(double targetDistance)
  {
    intakeController.setSetpoint(targetDistance);
  }

  public void getIntakePosition(double distance)
  {
    intakeController.setSetpoint(distance);
  }

  public void moveUp()
  {
    intakeMotor.set(1);
  }

  public void moveDown()
  {
    intakeMotor.set(-1);
  }

  public void stop()
  {
    intakeMotor.set(0);
  }

  public double getEncoderMeters()
  {
    return encoder.get() * encoderConversion; 
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
    intakeMotor.setVoltage(intakeController.calculate(intakeEncoder.getPosition()) + ff.calculate(encoderConversion, encoderConversion, encoderConversion));

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
