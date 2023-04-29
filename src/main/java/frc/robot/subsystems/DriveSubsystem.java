// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable{
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
    calibrateGyro();
    resetEncoders();
    leftMotorsGroup.setInverted(DriveConstants.kLeftMotorInverted);
    rightMotorsGroup.setInverted(DriveConstants.kRightMotorInverted);
    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    leftEncoder.setReverseDirection(DriveConstants.kEncoderLeftReversed);
    rightEncoder.setReverseDirection(DriveConstants.kEncoderRightReversed);
  }

  @Override
  public void close() throws Exception {
    leftMotor1.close();
    leftMotor2.close();
    rightMotor1.close();
    rightMotor2.close();
    leftMotorsGroup.close();
    rightMotorsGroup.close();
    driveTrain.close();
    leftEncoder.close();
    rightEncoder.close();
    navX.close();
  }

  @Override
  public void periodic() {
    if (getGyroIsConnected()) {
      field.setRobotPose(field.getRobotPose().getX(), field.getRobotPose().getY(), getGyroRotation2d());
    }
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of the motors
   * @param LeftMotorSpeed double between -1 and 1
   * @param RightMotorSpeed double between -1 and 1
   */
  public void setMotors(double LeftMotorSpeed, double RightMotorSpeed) {
    leftMotorsGroup.set(LeftMotorSpeed);
    rightMotorsGroup.set(RightMotorSpeed);
  }

  /**
   * Sets the speed of the motors
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
   */
  public void drive(double xSpeed, double zRotation) {
    driveTrain.arcadeDrive(-xSpeed, -zRotation);
  }

  /**
   * Sets the speed of the motors
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
   * @param squaredInputs If set, decreases the input sensitivity at low speeds.
   */
  public void drive(double xSpeed, double zRotation, boolean squaredInputs) {
    driveTrain.arcadeDrive(-xSpeed, -zRotation, squaredInputs);
  }

  public void stop() {
    driveTrain.stopMotor();
  }

  public void setEnabled(boolean enabled) {
    driveTrain.setSafetyEnabled(enabled);
  }

  /**
   * Resets the encoders
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Gets the distance the left encoder has traveled
   * @return double distance in meters
   */
  public double getLeftEncoderDistance() {
    return leftEncoder.getDistance();
  }

  /**
   * Gets the distance the right encoder has traveled
   * @return double distance in meters
   */
  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  /**
   * Gets the rate of the left encoder
   * @return double rate in meters per second
   */
  public double getLeftEncoderRate() {
    return leftEncoder.getRate();
  }

  /**
   * Gets the rate of the right encoder
   * @return double rate in meters per second
   */
  public double getRightEncoderRate() {
    return rightEncoder.getRate();
  }
  
  /**
   * Finds if the NavX is connected
   * @return boolean true if connected
   */
  public boolean getGyroIsConnected() {
    if (!navX.isConnected()) {
      DriverStation.reportError("NavX is not connected", false);
    }
    return navX.isConnected();
  }

  /**
   * Calibrates the navX
   * Doesn't do anything if the navX is not connected.
   */
  public void calibrateGyro() {
    if (!getGyroIsConnected()) {
      return;
    }
    navX.calibrate();
  }

  /**
   * Resets the Gyro Z (Yaw) axis to a heading of zero.
   * doesn't do anything if the navX is not connected.
   */
  public void resetGyro() {
    if (!getGyroIsConnected()) {
      return;
    }
    navX.reset();
  }

  /**
   * Gets the rotation of the navX in {@link Rotation2d}
   * @return Rotatation in terms of {@link Rotation2d}. returns null if navX is not connected.
   */
  public Rotation2d getGyroRotation2d() {
    if (!getGyroIsConnected()) {
      return null;
    }
    return navX.getRotation2d();
  }

  /**
   * Gets the rotation of the navX in degrees
   * NOTE: The angle is continuous, meaning it's range is beyond 360 degrees. 
   * This ensures that algorithms that wouldn't want to see a discontinuity in the gyro output 
   * as it sweeps past 0 on the second time around.
   * if you want it to be fixed use {@link DriveSubsystem#getGyroAngleFixed}
   * @return the total accumilated yaw angle (Z axis) double rotation in degrees. returns 0 if navX is not connected.
   */
  public double getGyroAngle() {
    if (!getGyroIsConnected()) {
      return 0.0;
    }
    return navX.getAngle();
  }

  /**
   * Gets the rotation of the navX in degrees but does not go past 360 degrees.
   * if you want it to be continuous use {@link DriveSubsystem#getGyroAngle}
   * @return the yaw angle (Z axis) double rotation in degrees. returns 0 if navX is not connected.
   */
  public double getGyroAngleFixed() {
    if (!getGyroIsConnected()) {
      return 0;
    }
    return navX.getAngle() % 360;
  }

  /**
   * Gets the rate of rotation of the navX in degrees per second
   * @return double rotation in degrees per second.  returns 0 if navX is not connected.
   */
  public double getGyroRate() {
    if (!getGyroIsConnected()) {
      return 0;
    }
    return navX.getRate();
  }

  /**
   * Gets if Gyro Is Calibrating
   * @return boolean true if calibrating. returns false if navX is not connected.
   */
  public boolean getGyroIsCalibrating() {
    if (!getGyroIsConnected()) {
      return false;
    }
    return navX.isCalibrating();
  }
}
