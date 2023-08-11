package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TopRollerSubsystem;

public class RunTopRollerCommand extends CommandBase{
    
    private double m_rollerSpeed;
    private boolean m_pistonOpen;

    private TopRollerSubsystem m_topRoller;

    public RunTopRollerCommand(TopRollerSubsystem topRollerSubsystem, double speed, boolean pistonOpen) {
        m_topRoller = topRollerSubsystem;
        m_rollerSpeed = speed;
        addRequirements(topRollerSubsystem);
    }

    @Override
    public void initialize() {
       m_topRoller.setRollerSpeed(m_rollerSpeed);
       if(m_pistonOpen) {
            m_topRoller.openPiston();
       } else {
            m_topRoller.closePiston();
       }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
