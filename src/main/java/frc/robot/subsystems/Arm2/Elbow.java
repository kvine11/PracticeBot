package frc.robot.subsystems.Arm2;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.ArmConstants;
//import frc.robot.Constants.ArmConstants.ElbowConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class Elbow extends SubsystemBase{
    public CANSparkMax RightElbowMotor;
    public AbsoluteEncoder ElbowEncoder;
    public ProfiledPIDController elbowController;
    private ArmFeedforward elbowFF;
    
    public Elbow()
    {
        RightElbowMotor.setInverted(false);
        ElbowEncoder = RightElbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
        ElbowEncoder.setPositionConversionFactor(2 * Math.PI * 1.0);
        elbowController = new ProfiledPIDController(0, 0, 0, null);
        elbowFF = new ArmFeedforward(0, 0, 0);
    }

    public double convertTickstoAngle(double angle)
    {
        double newAngle = angle;
        return newAngle;
    }

    public double getAngle()
    {
        return convertTickstoAngle(ElbowEncoder.getPosition());
    }

    public void setPosition(double position)
    {
        if(elbowController.getP() == 0)
        {
            elbowController.setP(5);
        }
        elbowController.setGoal(position);
    }

    public void setDistance(double distance)
    {
        elbowController.setGoal(distance);
    }

    


}
