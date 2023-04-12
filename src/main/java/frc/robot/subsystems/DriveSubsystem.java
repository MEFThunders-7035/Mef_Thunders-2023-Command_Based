// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(DriveConstants.kLeftMotor1Port);
  private final WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(DriveConstants.kLeftMotor2Port);
  
  private final WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(DriveConstants.kRightMotor1Port);
  private final WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(DriveConstants.kRightMotor2Port);

  MotorControllerGroup rightMotorsGroup = new MotorControllerGroup(rightMotor1, rightMotor2);
  MotorControllerGroup leftMotorsGroup = new MotorControllerGroup(leftMotor1, leftMotor2);
  
  DifferentialDrive driveTrain = new DifferentialDrive(leftMotorsGroup,rightMotorsGroup);
  
  public DriveSubsystem() {
    rightMotorsGroup.setInverted(true);
  }
  public void setMotors(double leftSpeed, double rightSpeed) {
    leftMotorsGroup.set(leftSpeed);
    rightMotorsGroup.set(rightSpeed);
  }
  public void drive(double speed, double rotation) {
    driveTrain.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
