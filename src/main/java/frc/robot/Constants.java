// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriveLeftMotor1 = 1;
    public static final int kDriveLeftMotor2 = 2;
    public static final int kDriveRightMotor1 = 3;
    public static final int kDriveRightmotor2 = 4;
    public static final int kArmMotor = 5;
    
  }

  public static final class ShooterConstants {
    public static double kSparkMaxP = 0.001;
    public static double kSparkMaxFeedforward = 0.000195; // .00022

    public static int kLeftMotorPort = 11;
    public static int kRightMotorPort = 12;

    public static double kVelocityTolerance = 115;
}

}
