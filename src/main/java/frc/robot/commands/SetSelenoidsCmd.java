package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
public class SetSelenoidsCmd extends CommandBase{
    private final PneumaticsSubsystem pneumaticsSubsystem;
    private final boolean selenoid_state;

    /**
     * Sets the Selenoids to the desired state
     * @param pneumaticsSubsystem the pneumatics subsystem
     * @param selenoid_state true = Push, false = Pull
     */
    public SetSelenoidsCmd(PneumaticsSubsystem pneumaticsSubsystem, boolean selenoid_state) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        this.selenoid_state = selenoid_state;
        addRequirements(pneumaticsSubsystem);
    }
    @Override
    public void execute() {
        pneumaticsSubsystem.setSolenoid(selenoid_state);
    }

}
