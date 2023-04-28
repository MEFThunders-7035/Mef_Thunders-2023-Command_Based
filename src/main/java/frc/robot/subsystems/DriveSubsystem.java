// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(DriveConstants.kLeftMotor1Port);
  private final WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(DriveConstants.kLeftMotor2Port);
  
  private final WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(DriveConstants.kRightMotor1Port);
  private final WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(DriveConstants.kRightMotor2Port);
  
  private final MotorControllerGroup rightMotorsGroup = new MotorControllerGroup(rightMotor1, rightMotor2);
  private final MotorControllerGroup leftMotorsGroup = new MotorControllerGroup(leftMotor1, leftMotor2);
  
  private final DifferentialDrive driveTrain = new DifferentialDrive(leftMotorsGroup,rightMotorsGroup);
  
  private final Encoder leftEncoder = new Encoder(DriveConstants.kEncoderLeftPort1, DriveConstants.kEncoderLeftPort2);
  private final Encoder rightEncoder = new Encoder(DriveConstants.kEncoderRightPort1, DriveConstants.kEncoderRightPort2);
  
  private final AHRS navX = new AHRS();
    
  private final Field2d field;
  
  public DriveSubsystem(Field2d field) {
    this.field = field;
    leftMotorsGroup.setInverted(DriveConstants.kLeftMotorInverted);
    rightMotorsGroup.setInverted(DriveConstants.kRightMotorInverted);
    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    leftEncoder.setReverseDirection(DriveConstants.kEncoderLeftReversed);
    rightEncoder.setReverseDirection(DriveConstants.kEncoderRightReversed);
  }

  @Override
  public void periodic() {
    if (getGyroIsConnected()) {
      field.setRobotPose(field.getRobotPose().getX(), field.getRobotPose().getY(), getGyroRotation2d());
    }
    // This method will be called once per scheduler run
  }

  public void setMotors(double LeftMotorSpeed, double RightMotorSpeed) {
    leftMotorsGroup.set(LeftMotorSpeed);
    rightMotorsGroup.set(RightMotorSpeed);
  }

  public void drive(double speed, double rotation) {
    driveTrain.arcadeDrive(-speed, -rotation);
  }

  public void stop() {
    driveTrain.stopMotor();
  }

  public void setEnabled(boolean enabled) {
    driveTrain.setSafetyEnabled(enabled);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftEncoderDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  public double getLeftEncoderRate() {
    return leftEncoder.getRate();
  }

  public double getRightEncoderRate() {
    return rightEncoder.getRate();
  }
  
  public boolean getGyroIsConnected() {
    return navX.isConnected();
  }

  public void calibrateGyro() {
    if (!getGyroIsConnected()) {
      throw new RuntimeException("NavX is not connected");
    }
    navX.calibrate();
  }

  public void resetGyro() {
    if (!getGyroIsConnected()) {
      throw new RuntimeException("NavX is not connected");
    }
    navX.reset();
  }

  public Rotation2d getGyroRotation2d() {
    if (!getGyroIsConnected()) {
      throw new RuntimeException("NavX is not connected");
    }
    return navX.getRotation2d();
  }

  public double getGyroAngle() {
    if (!getGyroIsConnected()) {
      throw new RuntimeException("NavX is not connected");
    }
    return navX.getAngle();
  }

  public double getGyroRate() {
    if (!getGyroIsConnected()) {
      throw new RuntimeException("NavX is not connected");
    }
    return navX.getRate();
  }

  public boolean getGyroIsCalibrating() {
    if (!getGyroIsConnected()) {
      throw new RuntimeException("NavX is not connected");
    }
    return navX.isCalibrating();
  }
}
