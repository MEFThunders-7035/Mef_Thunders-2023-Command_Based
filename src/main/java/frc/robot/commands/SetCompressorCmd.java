package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
public class SetCompressorCmd extends CommandBase{
    private final PneumaticsSubsystem pneumaticsSubsystem;
    private final boolean compressor_state;

    public SetCompressorCmd(PneumaticsSubsystem pneumaticsSubsystem, boolean compressor_state) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        this.compressor_state = compressor_state;
        addRequirements(pneumaticsSubsystem);
    }
    @Override
    public void execute() {
        pneumaticsSubsystem.setCompressor(compressor_state);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
