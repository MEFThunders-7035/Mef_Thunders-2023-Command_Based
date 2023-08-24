package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDriveCmd extends CommandBase{
    private final DriveSubsystem driveSubsystem;
    private final double speed;
    private final double time;
    private double first_heading;
    private PIDController headingPidController;
    private double fixheadingspeed;
    private Timer timer;
    private boolean finlished = false;
    
    /**
    * Drives foward for a certain amount of time
    * @param driveSubsystem the drive subsystem
    * @param speed the speed to drive at (between -1 and 1)
    * @param time the time to drive for (in seconds)
    * @throws InterruptedException if the thread is interrupted
    * @throws IllegalArgumentException if the speed is not between -1 and 1
    * @return the command
    */
    public TimedDriveCmd(DriveSubsystem driveSubsystem, double speed, double time) {
        this.driveSubsystem = driveSubsystem;
        this.speed = speed;
        this.time = time;
        this.timer = new Timer();
        headingPidController = new PIDController(
            AutonomousConstants.headingPIDConstants.kP,
            AutonomousConstants.headingPIDConstants.kI, 
            AutonomousConstants.headingPIDConstants.kD);
        addRequirements(driveSubsystem);
    }
    @Override
    public void initialize() {
        timer.reset();
        finlished = false;
        timer.start();
        first_heading = driveSubsystem.getGyroAngle();
        headingPidController.setSetpoint(first_heading);
    }
    @Override
    public void execute() {
        if (Math.abs(speed) > 1) throw new RuntimeException("Speed must be between -1 and 1");
        fixheadingspeed = headingPidController.calculate(driveSubsystem.getGyroAngle());
        if (timer.get() < time) {
            driveSubsystem.drive(speed, fixheadingspeed, false);
        }
        else {
            driveSubsystem.stop();
            finlished = true;
        }
    }
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0);
        if (interrupted) {
            System.out.println("TimedDriveCommand was interrupted");
            finlished = false;
        }
        else {
            System.out.println("TimedDriveCommand finished");
            finlished = true;
        }
    }
    @Override
    public boolean isFinished() {
        return finlished;
    }

}
