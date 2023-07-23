package frc.robot.autos;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.FieldCentricOffset;
import frc.robot.utilities.SwerveAutoBuilder;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*
 * <h3>PathPlannerCommand<h3>
 * 
 */
public class PathPlannerCommand extends SequentialCommandGroup {

    private static final double MAX_ACCELERATION = 2.5;
    private static final double MAX_VELOCITY = 1.0;

    /**
     * <h3>PathPlannerCommand</h3>
     * 
     * adding path constraints and builds auto command
     * 
     * @param preCommand a command that is run before the path starts
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to return a command that we want to execute at a marker
     * @param postCommand a command that is run after the path completes
     */
    public PathPlannerCommand(Command preCommand, SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap, Command postCommand) {
        addRequirements(s_Swerve);

        PathConstraints pathConstraints = PathPlanner.getConstraintsFromPath(pathName);
        if(pathConstraints == null) {
            pathConstraints = new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION);
        }

        // Load Path Group trajectory to follow. All units in meters.
        List<PathPlannerTrajectory> loadPathGroup = PathPlanner.loadPathGroup(pathName, 
             false, pathConstraints);

        // Adding a pre command to autonomous ex. autoBalance
        if(preCommand != null) {
            addCommands(preCommand);
        }
    }

    /**
     * 
     * <h3>PathPlannerCommand</h3>
     * 
     * Adds path constraints and builds auto command
     * 
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to return a command that we want to execute at a marker
     * @param postCommand a command that is run after the path completes
     */
    public PathPlannerCommand(SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap, Command postCommand) {
            this(null, s_Swerve, pathName, eventCommandMap, postCommand);
    }


    /**
     * <h3>PathPlannerCommand</h3>
     * 
     * Adds path constraints and builds auto command
     * 
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to return a command that we want to execute at a marker
     */
    public PathPlannerCommand(SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap) {
        this(s_Swerve, pathName, eventCommandMap, null);
    }

    /**
     * 
     * <h3>PathPlannerCommand</h3>
     * 
     * Adds path constraints and builds auto command
     * 
     * @param preCommand a command that is run before the path starts
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to return a command that we want to execute at a marker
     */
    public PathPlannerCommand(Command preCommand, SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap) {
        this(preCommand, s_Swerve, pathName, eventCommandMap, null);
    }
}