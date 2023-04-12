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
    public static final int kJoystickPort = 0;
  }
  public static class DriveConstants {
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 3;
    public static final int kRightMotor2Port = 4;
  }
  public static class VerticalElevatorConstants {
    public static final int kVerticalElevatorPort = 2;
    public static final double kSpeed = 0.6;
  }
  public static class HorizontalElevatorConstants {
    public static final int kHorizontalElevatorPort = 0;
  }
  public static class IntakeConstants {
    public static final int kRedlinePort = 1;
    public static final int kCanNeoPort = 5;
    public static class IntakePIDConstants {
      public static final double kP = 0.4;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }
  }
}
