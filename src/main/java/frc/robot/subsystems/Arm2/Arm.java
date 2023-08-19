package frc.robot.subsystems.Arm2;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
public class Arm extends SubsystemBase{
    private Shoulder shoulder = new Shoulder();
    private Elbow elbow = new Elbow();
    // private Telescope telescope = new Telescope();
    public Arm(){}
    public void setArmPos(double q1, double q2){
        elbow.setPosition(q1);
        shoulder.setPosition(q2);
    }

    
}//efiouayaeouifhuhdasd