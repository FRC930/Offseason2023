package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class SetArmDegreesCommand extends CommandBase{

    private ArmSubsystem m_arm;
    private double m_armPos;

    /**
     * <h3>SetArmDegreesCommand</h3>
     * 
     * Sets the positions of the shoulder and wrist motors to the desired positions.
     * @param armSubsystem The arm subsystem
     * @param manipulatorSubsystem The manipulator subsystem
     * @param armPosition The desired arm position in degrees
     * @param manipulatorPosition The desired manipulator position in degrees
     */
    public SetArmDegreesCommand(ArmSubsystem armSubsystem, double armPosition) {
        m_arm = armSubsystem;
        m_armPos = armPosition;
        addRequirements(armSubsystem);
    }


    @Override
    public void initialize() {
        if(m_arm != null) {
            m_arm.setPosition(m_armPos);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
