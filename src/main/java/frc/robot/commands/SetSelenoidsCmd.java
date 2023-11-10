package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
public class SetSelenoidsCmd extends CommandBase{
    private final PneumaticsSubsystem pneumaticsSubsystem;
    private final boolean selenoidState;

    /**
     * Sets the Selenoids to the desired state
     * @param pneumaticsSubsystem the pneumatics subsystem
     * @param selenoidState true = Push, false = Pull
     */
    public SetSelenoidsCmd(PneumaticsSubsystem pneumaticsSubsystem, boolean selenoidState) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        this.selenoidState = selenoidState;
        addRequirements(pneumaticsSubsystem);
    }
    @Override
    public void execute() {
        pneumaticsSubsystem.setSolenoid(selenoidState);
    }

}
