package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class Flywheel {
    private CANSparkMax flywheelMotor;
    private PIDController flywheelPID;
    private AbsoluteEncoder flywheelEncoder;
    private final double encoderConversion = 1 / 4096 * 0.1 * Math.PI;

    public Flywheel()
    {
        PIDController flywheelPID  = new PIDController(0, 0, 0);
        flywheelEncoder.setPositionConversionFactor(2 * Math.PI * 1.0);
    }

    public void setGoal(double position)
    {
        flywheelPID.setSetpoint(position);
    }

    public void Periodic()
    {
        flywheelMotor.setVoltage(flywheelPID.calculate(flywheelEncoder.getVelocity()));
    }
}
