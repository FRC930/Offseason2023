package frc.robot.utilities;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.armcommands.RunManipulatorRollerCommand;
import frc.robot.commands.armcommands.RunTopRollerCommand;
import frc.robot.commands.armcommands.SetArmDegreesCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;

public class CommandFactoryUtility {
    //Cube Ground Intake
    public static final double ARM_INTAKE_ANGLE = 235; //TODO find values

    //Stow Position
    public static double STOW_POSITION = 45.0;//70.0; //TODO find values

    //Low Score
    public static final double ARM_LOW_SCORE_ANGLE = STOW_POSITION; //STOW_POSITION;
    
    //Medium Score
    public static final double ARM_MEDIUM_SCORE_ANGLE = 90; //STOW_POSITION;

    //High Score
    public static final double ARM_HIGH_SCORE_ANGLE = 90; //STOW_POSITION;



    private CommandFactoryUtility() {}

    public static Command createIntakeCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        TopRollerSubsystem m_topRollerSubsystem) {
        final Command command = 
            new SetArmDegreesCommand(m_armSubsystem, ARM_INTAKE_ANGLE)
                .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.ROLLER_INTAKE_SPEED))
                .andThen(new RunTopRollerCommand(m_topRollerSubsystem, TopRollerSubsystem.ROLLER_INTAKE_SPEED));
        return command;
    }

    public static Command createStowArmCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        TopRollerSubsystem m_topRollerSubsystem) {
        final Command command = 
            new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.HOLD_SPEED)
            .andThen(new RunTopRollerCommand(m_topRollerSubsystem, 0.0))
            .andThen(new SetArmDegreesCommand(m_armSubsystem, STOW_POSITION))
            .andThen(m_armSubsystem.createWaitUntilLessThanAngleCommand(170.0))    
            .andThen(m_armSubsystem.createWaitUntilGreaterThanAngleCommand(45.0));
        return command;
    }

    public static Command createScoreCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        double m_armPosition,
        double m_manipulatorspeed,
        double m_timeOut) {
        Command command;

        command = 
            new SetArmDegreesCommand(m_armSubsystem, m_armPosition)
            .andThen(m_armSubsystem.createWaitUntilAtAngleCommand().withTimeout(m_timeOut))
            .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, m_manipulatorspeed));
        return command;
    }

    public static Command createScoreLowCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {

        return createScoreCommand(m_armSubsystem, m_manipulatorSubsystem, ARM_LOW_SCORE_ANGLE, ManipulatorSubsystem.LOW_SCORE_SPEED, 0.5);
    }

    public static Command createScoreMediumCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {

        return createScoreCommand(m_armSubsystem, m_manipulatorSubsystem, ARM_MEDIUM_SCORE_ANGLE, ManipulatorSubsystem.MEDIUM_SCORE_SPEED, 0.5);
    }

    public static Command createScoreHighCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {

        return createScoreCommand(m_armSubsystem, m_manipulatorSubsystem, ARM_HIGH_SCORE_ANGLE, ManipulatorSubsystem.HIGH_SCORE_SPEED, 0.5);
    }

    /**
     * addAutoCommandEvent
     *  Add event command to event map for autonomous paths
     * 
     * @param eventCommandMap
     * @param eventName
     * @param m_elevatorSubsystem
     * @param m_armSubsystem
     * @param m_manipulatorSubsystem
     */
    public static void addAutoCommandEvent( Map<String, Command> eventCommandMap, 
                                            String eventName, 
                                            ArmSubsystem m_armSubsystem, 
                                            ManipulatorSubsystem m_manipulatorSubsystem) {
        Command autoCommand = null;
        switch(eventName) {
        }

        if(autoCommand != null) {
            eventCommandMap.put(eventName, autoCommand);
        } else {
            DriverStation.reportWarning("Unable to add event name"+eventName+" given not declared!", 
                Thread.currentThread().getStackTrace());
        }
     
    }

}