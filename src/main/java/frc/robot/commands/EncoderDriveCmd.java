package frc.robot.commands;

import frc.robot.Constants.AutonomousConstants.EncoderPIDConstants;
import frc.robot.Constants.AutonomousConstants.headingPIDConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class EncoderDriveCmd extends CommandBase {
  private final DriveSubsystem driveSubsystem;
	private final PIDController headingPidController;
	private final PIDController EncoderPIDController;
  private final double distance;
	private double first_heading;
	private boolean finished;

  /**
   * Creates a new EncoderDriveCmd.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public EncoderDriveCmd(DriveSubsystem driveSubsystem, double distance) {
    this.driveSubsystem = driveSubsystem;
    this.distance = distance;
		headingPidController = new PIDController(
            headingPIDConstants.kP,
            headingPIDConstants.kI, 
            headingPIDConstants.kD);
		EncoderPIDController = new PIDController(
						EncoderPIDConstants.kP,
						EncoderPIDConstants.kI, 
						EncoderPIDConstants.kD);
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    driveSubsystem.resetEncoders();
    headingPidController.setSetpoint(first_heading);
		EncoderPIDController.setSetpoint(distance);
		finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		if (finished) {
			return;
		}
		double fixheadingspeed = headingPidController.calculate(driveSubsystem.getGyroAngle());
		double fixdistancespeed = EncoderPIDController.calculate(driveSubsystem.getAvarageEncoderDistance());

		driveSubsystem.drive(fixdistancespeed, fixheadingspeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}

