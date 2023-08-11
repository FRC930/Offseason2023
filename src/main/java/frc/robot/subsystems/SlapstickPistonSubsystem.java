package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class SlapstickPistonSubsystem extends SubsystemBase{

    private final Solenoid m_slapstick;

    public SlapstickPistonSubsystem(int solenoid_ID) {
        
        //CTRE pneumatic hub has 8 slots. Cap is placed on simulation to prevent errors.
        m_slapstick = new Solenoid(
            Robot.isReal() || solenoid_ID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM, 
            solenoid_ID);
    }

    public void toggle() {
        m_slapstick.set(!m_slapstick.get());
    }
    
    public boolean isOpen() {
        return m_slapstick.get();
    }

    public void open() {
        m_slapstick.set(true);
    }

    public void close() {
        m_slapstick.set(false);
    }
}