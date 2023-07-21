package frc.robot.utilities;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.armcommands.RunManipulatorRollerCommand;
import frc.robot.commands.armcommands.SetArmDegreesCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CommandFactoryUtility {
    private final static double MANIPULATOR_REDUCTION = 0.0;

    //Cube Ground Intake
    public static final double ARM_INTAKE_ANGLE = -24.0; //TODO find values
    public static final double MANIPULATOR_INTAKE = 17.0 + MANIPULATOR_REDUCTION - 4; //TODO find values

    //Low Score
    public static final double ARM_LOW_SCORE_ANGLE = 70.0;
    public static final double MANIPULATOR_LOW_SCORE = -25.0 + MANIPULATOR_REDUCTION;



    private CommandFactoryUtility() {}

    public static Command createScoreCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        double waitSecondAfterElevator,
        double armPosition,
        double manipulatorPosition,
        double waitSecondArm) {
        Command command;
        
        command = new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, armPosition);
        // iF wish to wait for arm/manipulator gets to position than release or DONT release DRIVER will control this
        if(waitSecondArm >= 0.0) {
            command = command
            .andThen(m_armSubsystem.createWaitUntilAtAngleCommand()
                .withTimeout(waitSecondArm/2.0))
            .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.RELEASE_SPEED)
            );
        }

        return command;
    }
   
    public static Command createScoreLowCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        boolean releaseAtEnd) {
        return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            1.0, 
            ARM_LOW_SCORE_ANGLE, 
            MANIPULATOR_LOW_SCORE, 
            releaseAtEnd?1.5:-1.0);
    }

    public static Command createStowArmCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        final Command command = 
            new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.HOLD_SPEED)
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, 
                ArmSubsystem.STOW_POSITION))
            .andThen(m_armSubsystem.createWaitUntilLessThanAngleCommand(170.0))    
            .andThen(m_armSubsystem.createWaitUntilGreaterThanAngleCommand(45.0));
        return command;
    }


    private static Command createArmIntakeCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        double elevatorHeight,
        double waitSecond,
        double armPosition,
        double manipulatorPosition) {
        Command command;
        command = new SequentialCommandGroup(
            new RunManipulatorRollerCommand(m_manipulatorSubsystem,
                ManipulatorSubsystem.ROLLER_INTAKE_SPEED),
            // m_elevatorSubsystem.createWaitUntilAtHeightCommand().withTimeout(waitSecond),
            new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, armPosition));

        return command;
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
                                            ElevatorSubsystem m_elevatorSubsystem, 
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