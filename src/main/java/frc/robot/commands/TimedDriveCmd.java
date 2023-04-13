package frc.robot.commands;

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
        new Thread(() -> {
            try {
                driveSubsystem.drive(speed, 0);
                Thread.sleep((long) (time * 1000));
                driveSubsystem.drive(0, 0);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            finlished = true;
        }).start();
    }
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0);
        if (interrupted) {
            System.out.println("DriveBackwardsCmd was interrupted");
            finlished = false;
        }
        else {
            System.out.println("DriveBackwardsCmd finished");
            finlished = true;
        }
    }
    @Override
    public boolean isFinished() {
        return finlished;
    }

}
