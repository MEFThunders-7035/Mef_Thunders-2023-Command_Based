// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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
    public static final int kLeftMotor1Port = 3;
    public static final int kLeftMotor2Port = 4;
    public static final int kRightMotor1Port = 1;
    public static final int kRightMotor2Port = 2;

    public static final boolean kLeftMotorInverted = false;
    public static final boolean kRightMotorInverted = true;

    public static final int kEncoderLeftPort1 = 2;
    public static final int kEncoderLeftPort2 = 3;
    public static final int kEncoderRightPort1 = 4;
    public static final int kEncoderRightPort2 = 5;

    public static final boolean kEncoderLeftReversed = false;
    public static final boolean kEncoderRightReversed = false;

    public static final int kEncoderDistancePerPulse = 1;
  }
  
  public static class VerticalElevatorConstants {
    public static final int kVerticalElevatorPort = 0;
    public static final double kSpeed = 0.6;
  }

  public static class PneumaticsConstants {
    public static final int kCompressorPort = 0;
    public static final int kSolenoidFowardPort = 0;
    public static final int kSolenoidReversePort = 1;
  }

  public static class IntakeConstants {
    public static final int kRedlinePort = 1;
    public static final int kIntakeArmPotPort = 0;
    public static final int kCanIntakeArmMotor1Port = 5;
    public static final int kCanIntakeArmMotor2Port = 2;
    public static final double kUpSpeed = 0.5;
    public static final double kDownSpeed = -0.3;
    public static final double kRedlineSpeed = 0.5;
    public static class IntakePIDConstants {
      public static final double kP = 0.4;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }
  }
  public static class IoConstants {
    public static final int kJoystickPort = 0;
    public static final int X_AXIS = 0;
    public static final int Y_AXIS = 1;
    public static final int Z_AXIS = 2;
  }
  public static class AutonomousConstants {
    public static final double kDriveSpeed = -0.5;
    public static final double kDriveTime = 3;
    public static final double kRedlineSpeed = 0.1;
    public static final double kRedlineTime = 1;

    public static final String kTimedAuto = "Timer Auto";
    public static final String kGyroAuto = "Gyro Auto";
    public static final String kCameraAuto = "Camera Auto";
    public static final String kStabilize = "Stabilize Auto";
  }

  public static class PhotonVisionConstants {
    public static class Cameras {
      public static final String kPiCamera = "IMX219";
      public static final String kWideCamera = "HD_Pro_Webcam_C920";
    }

    public static final double kTargetArea = 0.5;
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(15.13);
    
    public static class PiCamera{
      public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(1);
      public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);  
      public static final Transform3d robotToCam = new Transform3d(
          new Translation3d(0.5, 0.0, 0),
          new Rotation3d(0, 0,0));

          public static class TurnPIDConstants{
            public static final double kP = 0.05;
            public static final double kI = 0;
            public static final double kD = 0;
          }
          public static class FowardPIDConstants {
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;
          }
    }
    
    public static class WideCamera{
      public static final double kCamera_Height_Meters = 1.15;
      /**
       * The angle of the camera relative to the ground in radians counterclockwise is positive.
       */
      public static final double kCamera_Pitch_Radians = Units.degreesToRadians(30);  
      public static final Transform3d robotToCam = new Transform3d(
          new Translation3d(-0.1, kCamera_Height_Meters, 0),
          new Rotation3d(0, Units.degreesToRadians(30), 0));

          public static class TurnPIDConstants{
            public static final double kP = 0.02;
            public static final double kI = 0;
            public static final double kD = 0;
          }
          public static class FowardPIDConstants {
            public static final double kP = 0.05;
            public static final double kI = 0;
            public static final double kD = 0;
          }
    }
    
    
  }
}
