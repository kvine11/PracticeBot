package frc.robot.subsystems;


import com.revrobotics.*;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private CANSparkMax flywheelMotor1;
    private CANSparkMax flywheelMotor2;
    private PIDController flywheelPID;
    private AbsoluteEncoder flywheelEncoder;
    private SimpleMotorFeedforward ff;
    //private final double encoderConversion = 1 / 4096 * 0.1 * Math.PI;

    public Flywheel()
    {
        flywheelMotor1 = new CANSparkMax(11, MotorType.kBrushless);
        flywheelMotor2 = new CANSparkMax(12, MotorType.kBrushless); //tbd need to add ports
        flywheelPID = new PIDController(ShooterConstants.kSparkMaxP, 0, 0);
        flywheelEncoder = flywheelMotor1.getAbsoluteEncoder(Type.kDutyCycle);
        flywheelEncoder.setPositionConversionFactor(2 * Math.PI * 1.0);
        flywheelMotor2.follow(flywheelMotor1, true);
        ff = new SimpleMotorFeedforward(ShooterConstants.kSparkMaxFeedforward, 0);

    }

    public void setGoal(double velocity)
    {
        flywheelPID.setSetpoint(velocity);
    }

    @Override
    public void periodic()
    {
        double velocity = flywheelPID.calculate(flywheelEncoder.getPosition()) + ff.calculate(ShooterConstants.kSparkMaxFeedforward);
        flywheelMotor1.setVoltage(velocity);
        SmartDashboard.putNumber("Flywheel velocity", velocity);
    }

    //add two motors V
    //call in robot container V
    //use feedforward
    // use sparkMax pid if wpilib no work
}
