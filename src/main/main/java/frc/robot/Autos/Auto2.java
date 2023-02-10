package frc.robot.Autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto2 extends SequentialCommandGroup {
    


    public  Auto2(Swerve s_Swerve){
    // Path Planner Path
    String robot_path = "Auto2";
    PathPlannerTrajectory TestPath = PathPlanner.loadPath(robot_path, new PathConstraints(2, .5));

    HashMap<String, Command> eventMap = new HashMap<>();

   // eventMap.put("Intake", new IntakeCommand(m_IntakeSubsystem));
   // eventMap.put("IntakeOff", new IntakeOffCommand(m_IntakeSubsystem));

    // eventMap.put("IntakeArmsDown", new PrintCommand("intakeArmsDown"));

    // 4. Construct command to follow trajectory

    // auto builder can use events added in through Path Planner
    SwerveAutoBuilder autobuilder = new SwerveAutoBuilder(
        s_Swerve::getPose,
        s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics,
        new PIDConstants(.9, 0, .01),
        new PIDConstants(1, 0, .01),
        s_Swerve::setModuleStates,
        eventMap,
        s_Swerve);

    Command fullAuto = autobuilder.fullAuto(TestPath);


    // 5. Add some init and wrap-up, and return everything
    addCommands(
     new SequentialCommandGroup(
        // new InstantCommand(() ->
        // swerveSubsystem.resetOdometry(TestPath.getInitialPose())),
        fullAuto,
        new InstantCommand(() -> s_Swerve.stopModules())));

    // new InstantCommand(() -> swerveSubsystem.getPose()));
  }
}