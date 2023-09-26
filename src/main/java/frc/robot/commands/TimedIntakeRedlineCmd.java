package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Redline_IntakeSubsystem;

public class TimedIntakeRedlineCmd extends CommandBase{
    private final Redline_IntakeSubsystem RedlineIntakesubsystem;
    private final double speed;
    private final double time;
    private Timer timer;
    private boolean finished = false;

    /**
     * Runs the intake for a specified amount of time
     * @param RedlineIntakesubsystem The intake subsystem
     * @param speed The speed to run the intake at (-1 to 1)
     * @param time The time to run the intake for in seconds
     */
    public TimedIntakeRedlineCmd(Redline_IntakeSubsystem RedlineIntakesubsystem, double speed, double time) {
        this.RedlineIntakesubsystem = RedlineIntakesubsystem;
        this.speed = speed;
        this.time = time;
        this.timer = new Timer();
        addRequirements(RedlineIntakesubsystem);
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
        RedlineIntakesubsystem.setRedline(speed);
    }

    @Override
    public void end(boolean interrupted) {
        RedlineIntakesubsystem.setRedline(0);
        timer.stop(); timer.reset();
        System.out.println("TimedIntakeRedlineCmd ended " + (interrupted ? "interruptedly" : "normally"));
    }
    
    @Override
    public boolean isFinished() {
        return finished;
    }
}
