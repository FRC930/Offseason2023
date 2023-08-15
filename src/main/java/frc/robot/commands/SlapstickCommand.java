package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SlapstickSubsystem;

public class SlapstickCommand extends CommandBase {
    
    private final boolean open;

    private final SlapstickSubsystem m_piston;

    public SlapstickCommand(boolean pistonOpen, SlapstickSubsystem slapstickSubsystem) {
        open = pistonOpen;
        m_piston = slapstickSubsystem;

        addRequirements(m_piston);
    }

    @Override
    public void initialize() {
        if(open) {
            m_piston.open();
        } else {
            m_piston.close();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
