// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.VerticalElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class VerticalElevatorJoystickCmd extends CommandBase {

  private final VerticalElevatorSubsystem verticalElevatorSubsytem;
  private final double speed;

  public VerticalElevatorJoystickCmd(VerticalElevatorSubsystem subsystem, double speed) {
    this.verticalElevatorSubsytem = subsystem;
    this.speed = speed;
    addRequirements(verticalElevatorSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // we don't need to initialize anything
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (speed > 0) {
      if (verticalElevatorSubsytem.getTopLimitSwitch())
        verticalElevatorSubsytem.setMotor(0);
      else
        verticalElevatorSubsytem.setMotor(speed);
    } else {
      if (verticalElevatorSubsytem.getBottomLimitSwitch())
        verticalElevatorSubsytem.setMotor(0);
      else
        verticalElevatorSubsytem.setMotor(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    verticalElevatorSubsytem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
