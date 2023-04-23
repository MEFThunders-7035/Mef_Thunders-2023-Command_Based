package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDriveCmd extends CommandBase{
    private final DriveSubsystem driveSubsystem;
    private final double speed;
    private final double time;
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
        addRequirements(driveSubsystem);
    }
    @Override
    public void initialize() {
        finlished = false;
    }
    @Override
    public void execute() {
        if (Math.abs(speed) > 1) throw new IllegalArgumentException("Speed must be between -1 and 1");
        new Thread(() -> {
            try {
                System.out.println("TimedDriveCommand started");
                driveSubsystem.drive(speed, 0);
                System.out.println("TimedDriveCommand waiting for " + time + " seconds");
                Thread.sleep((long) (time * 1000));
                driveSubsystem.drive(0, 0);
            } catch (InterruptedException e) {
                driveSubsystem.setMotors(0, 0);
                DriverStation.reportError("Auto Command interrupted", e.getStackTrace());
                e.printStackTrace();
                throw new RuntimeException(e);
            }
            finlished = true;
        }).start();
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
