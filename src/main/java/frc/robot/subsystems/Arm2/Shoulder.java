package frc.robot.subsystems.Arm2;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.ArmConstants.ShoulderConstants;
//import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;

public class Shoulder extends SubsystemBase{
    public CANSparkMax leftShoulder;
    public CANSparkMax rightShoulder;
    public AbsoluteEncoder shoulderEncoder;
    public ProfiledPIDController shoulderController;
    private ArmFeedforward shoulderFF;

    public Shoulder()
    {
        leftShoulder.follow(rightShoulder, true);
        rightShoulder.setInverted(false);
        shoulderEncoder = rightShoulder.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderEncoder.setPositionConversionFactor(2 * Math.PI * 1.0);
        shoulderController = new ProfiledPIDController(0, 0, 0, null);
        shoulderFF = new ArmFeedforward(0, 0, 0);



    }

    public double convertTickstoAngle(double angle)
    {
        double newAngle = angle;
        return newAngle;
    }

    public void setPosition(double goal)
    {
        if(shoulderController.getP() == 0)
        {
            shoulderController.setP(5);
        }
        shoulderController.setGoal(goal);
    }

    public double getAngle()
    {
        return convertTickstoAngle(shoulderEncoder.getPosition());
    }
}
