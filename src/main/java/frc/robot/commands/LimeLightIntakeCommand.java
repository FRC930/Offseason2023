package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.GamePieceDetectionUtility;

/**
 * 
 * <h3>LimeLightIntakeCommand</h3>
 * 
 * Uses game piece detection and a Trapezoidal Profile to smoothly and accurately intake a cube during autonomous
 * 
 */
public class LimeLightIntakeCommand extends CommandBase {
    private final double MAX_STRAFE = 0.1; //TODO tune this value on the robot. Tune PID value first and set this value as a hard stop to prevent outlying data
    private PIDController pid = new PIDController(0.01, 0.0, 0.0); //(0.01, 0.0, 0.0); //TODO tune this value

    private SwerveDrive m_SwerveDrive;
    private GamePieceDetectionUtility m_LimeLight;

    private Pose2d m_position;
    
    private double m_throttle;
    private double m_strafe;

    private double m_distance;
    private double m_TimeElapsed = 0.0;

    private final TrapezoidProfile.Constraints m_constraints = //maximum velocity and acceleration for the command
        new TrapezoidProfile.Constraints(0.25, 0.25);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;
    private TrapezoidProfile profile;


    /**
     * <h3>LimeLightIntakeCommand</h3>
     * 
     * Uses game piece detection and a Trapezoidal Profile to smoothly and accurately intake a cube during autonomous
     * 
     * @param swerveDrive Swerve Drive
     * @param limeLight GamePieceDetectionUtility
     * @param position Pose2d of the location of where the robot should go
     * 
     */
    public LimeLightIntakeCommand(SwerveDrive swerveDrive, GamePieceDetectionUtility limeLight, Pose2d position) {
        m_SwerveDrive = swerveDrive;
        m_LimeLight = limeLight;
        m_position = position;
        addRequirements(m_SwerveDrive);
    }

    @Override
    public void initialize() { 
        m_distance = Math.sqrt( //Uses the Pythagorean Theorem to calculate the total distance to the target
            Math.pow(
                Math.abs(m_position.getX() - m_SwerveDrive.getPose().getX()), 
                2.0
            )
            +
            Math.pow(
                Math.abs(m_position.getY() - m_SwerveDrive.getPose().getY()),
                2.0
            )
        );

        //Creates the trapezoid profile using the given information
        m_goal = new TrapezoidProfile.State(m_distance, 0); //sets the desired state to be the total distance away
        m_setpoint = new TrapezoidProfile.State(0,0); //sets the current state at (0,0)
        profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint); //combines everything into the trapezoid profile
    }

    @Override
    public void execute() {
        //uses a clamp and pid on the game piece detection camera to figure out the strafe (left & right)
        m_strafe = MathUtil.clamp(pid.calculate(m_LimeLight.get_tx(), 0.0), -MAX_STRAFE, MAX_STRAFE); 
        m_setpoint = profile.calculate(m_TimeElapsed);
        m_throttle = m_setpoint.velocity; //sets the throttle (speed) to  the current point on the trapezoid profile
        m_TimeElapsed += 0.02; //increases the timer  by 20 milliseconds

        SmartDashboard.putNumber("throttle", m_throttle);
        SmartDashboard.putNumber("strafe", m_strafe);

        /* 
         * This command utilitzes the swerve drive while it isn't field relative. 
         * The swerve drive returns back to field relative after the command is used.
         * This is located in RobotContainer at line 155
        */
        m_SwerveDrive.drive(m_throttle, m_strafe, 0.0, false, true);
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished(m_TimeElapsed); //ends the command when the timer reaches the end of the trapezoid profile
    }
}
