// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  
  private final Gyro mpu6050;
  private final I2C.Port port;
  
  private final Field2d field;
  
  public DriveSubsystem(Field2d field) {
    this.port = I2C.Port.kOnboard;
    this.mpu6050 = new AHRS(port);
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
    mpu6050.close();
  }

  @Override
  public void periodic() {
    field.setRobotPose(field.getRobotPose().getX(), field.getRobotPose().getY(), getGyroRotation2d());
    // SmartDashboard.putNumber("Rotation", getGyroAngle());
    //SmartDashboard.putNumber("Left Encoder Distance", getLeftEncoderDistance());
    //SmartDashboard.putNumber("Right Encoder Distance", getRightEncoderDistance());
  }

  /**
   * Sets the speed of the motors
   * @param LeftMotorSpeed double between -1 and 1
   * @param RightMotorSpeed double between -1 and 1
   */
  public void setMotors(double LeftMotorSpeed, double RightMotorSpeed) {
    if (Math.abs(LeftMotorSpeed) > 1 || Math.abs(RightMotorSpeed) > 1) {
      DriverStation.reportError("Speed must be between -1 and 1", false);
      throw new IllegalArgumentException("Speed must be between -1 and 1");
    }
    leftMotorsGroup.set(LeftMotorSpeed);
    rightMotorsGroup.set(RightMotorSpeed);
  }

  /**
   * Sets the speed of the motors
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
   */
  public void drive(double xSpeed, double zRotation) {
    if (Math.abs(xSpeed) > 1 || Math.abs(zRotation) > 1) {
      DriverStation.reportError("Speed must be between -1 and 1", false);
      throw new IllegalArgumentException("Speed must be between -1 and 1");
    }
    driveTrain.arcadeDrive(-xSpeed, -zRotation);
  }

  /**
   * Sets the speed of the motors
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
   * @param squaredInputs If set, decreases the input sensitivity at low speeds.
   */
  public void drive(double xSpeed, double zRotation, boolean squaredInputs) {
    if (Math.abs(xSpeed) > 1 || Math.abs(zRotation) > 1) {
      DriverStation.reportError("Speed must be between -1 and 1", false);
    }
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
   * Calibrates the navX
   * Doesn't do anything if the navX is not connected.
   */
  public void calibrateGyro() {
    mpu6050.calibrate();
  }

  /**
   * Reset the gyro. Resets the gyro to a heading of zero. 
   * This can be used if there is significant drift in the gyro, 
   * and it needs to be recalibrated after it has been running.
   */
  public void resetGyro() {
    mpu6050.reset();
  }

  /**
   * Gets the rotation of the navX in {@link Rotation2d}
   * @return Rotatation in terms of {@link Rotation2d}.
   */
  public Rotation2d getGyroRotation2d() {
    return mpu6050.getRotation2d();
  }

  /**
   * Gets the rotation of the navX in degrees
   * NOTE: The angle is continuous, meaning it's range is beyond 360 degrees. 
   * This ensures that algorithms that wouldn't want to see a discontinuity in the gyro output 
   * as it sweeps past 0 on the second time around.
   * if you want it to be fixed use {@link DriveSubsystem#getGyroAngleFixed}
   * @return the total accumilated yaw angle (Z axis) double rotation in degrees.
   */
  public double getGyroAngle() {
    return mpu6050.getAngle();
  }

  /**
   * Gets the rotation of the navX in degrees but does not go past 360 degrees.
   * if you want it to be continuous use {@link DriveSubsystem#getGyroAngle}
   * @return the yaw angle (Z axis) double rotation in degrees.
   */
  public double getGyroAngleFixed() {
    return mpu6050.getAngle() % 360;
  }

  /**
   * Gets the rate of rotation of the navX in degrees per second
   * @return double rotation in degrees per second. 
   */
  public double getGyroRate() {
    return mpu6050.getRate();
  }
}
