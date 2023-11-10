package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
public class SetCompressorCmd extends CommandBase{
    private final PneumaticsSubsystem pneumaticsSubsystem;
    private final boolean compressorState;

    public SetCompressorCmd(PneumaticsSubsystem pneumaticsSubsystem, boolean compressorState) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        this.compressorState = compressorState;
        addRequirements(pneumaticsSubsystem);
    }
    @Override
    public void execute() {
        pneumaticsSubsystem.setCompressor(compressorState);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
