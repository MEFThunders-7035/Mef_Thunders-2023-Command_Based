package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RedlineIntakeSubsystem;

public class TimedIntakeRedlineCmd extends CommandBase{
    private final RedlineIntakeSubsystem redlineIntakeSubsystem;
    private final double speed;
    private final double time;
    private Timer timer;
    private boolean finished = false;

    /**
     * Runs the intake for a specified amount of time
     * @param redlineIntakeSubsystem The intake subsystem
     * @param speed The speed to run the intake at (-1 to 1)
     * @param time The time to run the intake for in seconds
     */
    public TimedIntakeRedlineCmd(RedlineIntakeSubsystem redlineIntakeSubsystem, double speed, double time) {
        this.redlineIntakeSubsystem = redlineIntakeSubsystem;
        this.speed = speed;
        this.time = time;
        this.timer = new Timer();
        addRequirements(redlineIntakeSubsystem);
    }

    @Override
    public void initialize() {
        finished = false;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() > time) {
            finished = true;
            return;
        }
        redlineIntakeSubsystem.setRedline(speed);
    }

    @Override
    public void end(boolean interrupted) {
        redlineIntakeSubsystem.setRedline(0);
        timer.stop(); timer.reset();
        System.out.println("TimedIntakeRedlineCmd ended " + (interrupted ? "interruptedly" : "normally"));
    }
    
    @Override
    public boolean isFinished() {
        return finished;
    }
}
