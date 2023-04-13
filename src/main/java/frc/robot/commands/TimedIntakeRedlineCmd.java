package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Redline_IntakeSubsystem;

public class TimedIntakeRedlineCmd extends CommandBase{
    private final Redline_IntakeSubsystem RedlineIntakesubsystem;
    private final double speed;
    private final double time;
    private boolean finished = false;

    public TimedIntakeRedlineCmd(Redline_IntakeSubsystem RedlineIntakesubsystem, double speed, double time) {
        this.RedlineIntakesubsystem = RedlineIntakesubsystem;
        this.speed = speed;
        this.time = time;
        addRequirements(RedlineIntakesubsystem);
    }
    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        new Thread(() -> {
            try {
                RedlineIntakesubsystem.setRedline(speed);
                Thread.sleep((long) (time * 1000));
                RedlineIntakesubsystem.setRedline(0);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            finished = true;
        }).start();
    }

    @Override
    public void end(boolean interrupted) {
        RedlineIntakesubsystem.setRedline(0);
        if (interrupted) {
            System.out.println("TimedIntakeRedlineCmd was interrupted!");
        }
        else {
            System.out.println("TimedIntakeRedlineCmd finished!");
            finished = true;
        }
    }
    
    @Override
    public boolean isFinished() {
        return finished;
    }
}
