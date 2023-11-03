package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.*;
import java.util.*;

public class Auto {
    PathPlannerTrajectory twoPiece = PathPlanner.loadPath("2 piece", new PathConstraints(4 , 3));  
    HashMap<String, InstantCommand> eventMap = new HashMap<>();

    public Auto(Arm m_Arm)
    {
        eventMap.put("cube 1" , new PrintCommand("passed cube 1"));
        eventMap.put("arm up", new InstantCommand(() -> m_Arm.setArmPosition(Units.degreesToRadians(90))));
        eventMap.put("c1armdown",new InstantCommand(() -> m_Arm.setArmPosition(Units.degreesToRadians(90)))); 
        eventMap.put("cube 2" , new PrintCommand("passed cube 2"));
        eventMap.put("arm up", new InstantCommand(() -> m_Arm.setArmPosition(Units.degreesToRadians(90))));
        eventMap.put("c2armdown" , new InstantCommand(() -> m_Arm.setArmPosition(Units.degreesToRadians(90))));
    }

}
 