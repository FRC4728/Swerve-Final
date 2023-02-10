package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.opencv.photo.Photo;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autos.Auto1;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0 ;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 2);
    private final JoystickButton Teleop1 = new JoystickButton(driver, 3);
    private final JoystickButton See = new JoystickButton(driver, 1);


    /* Subsystems */
 
    private final PhotonVisionSubsystem s_Vision = new PhotonVisionSubsystem();
    private final Swerve s_Swerve = new Swerve(s_Vision);
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();


    
    double r;
    double Theta;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis)
            )
        );


          m_chooser.setDefaultOption("Auto1", new Auto1(s_Swerve));
 //    m_chooser.addOption("Complex Auto", m_complexAuto);

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        See.onTrue(new InstantCommand(() -> s_Vision.CameraGet()));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
            Teleop1.onTrue(ramseteTeleopCommand(new Pose2d(Units.inchesToMeters(570), Units.inchesToMeters(42.19), new Rotation2d((0) * Math.PI))));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_chooser.getSelected();
    }

// create a path to run on chooser
public Command AutoTemplate() {
    // Path Planner Path
    String robot_path = "TestPath";
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
    return new SequentialCommandGroup(
        // new InstantCommand(() ->
        // swerveSubsystem.resetOdometry(TestPath.getInitialPose())),
        fullAuto,
        new InstantCommand(() -> s_Swerve.stopModules()));

    // new InstantCommand(() -> swerveSubsystem.getPose()));
  }

  Pose2d updateangle(double D, double Angle, Rotation2d targetAngle) {

    Pose2d translate;
    double currentangle = s_Swerve.getTheta();
      double ChangedAngle =  Angle - currentangle;

      double newX = D * Math.cos(ChangedAngle);
      double newY = D * -Math.sin(ChangedAngle);
      translate = new Pose2d(newX, newY, targetAngle);
      //SmartDashboard.putString("New Trajectory", translate.toString());

     

    return translate;
  }

  Trajectory newTrajectory(TrajectoryConfig config, Translation2d translation, Translation2d startingPose, Rotation2d angle, Rotation2d newAngle) {

    var interiorQuinticWaypoints = new ArrayList<Pose2d>();
    interiorQuinticWaypoints.add(new Pose2d(startingPose, angle));

    interiorQuinticWaypoints.add(new Pose2d(translation, newAngle));


    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(interiorQuinticWaypoints, config);
    return trajectory;
  }

  // Teleop Ramsete Builder: creates paths and follows them
  Command ramseteTeleopCommand(Pose2d targetPose2d) {
    TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics( Constants.Swerve.swerveKinematics
                );

    //    r = Math.sqrt((targetPose2d.getX()* targetPose2d.getX()) + targetPose2d.getY() * targetPose2d.getY());
      //  Theta = Math.atan(targetPose2d.getY() / targetPose2d.getX());

      //  updateangle(r, Theta, targetPose2d.getRotation());

      Trajectory trajectory =  newTrajectory(config, targetPose2d.getTranslation(), s_Vision.CameraGet(), s_Swerve.getYaw(), targetPose2d.getRotation());

                var interiorQuinticWaypoints = new ArrayList<Pose2d>();
                interiorQuinticWaypoints.add(new Pose2d(0, 0,new Rotation2d(0)));
                interiorQuinticWaypoints.add(new Pose2d(1,1, new Rotation2d(0)));
              //  interiorQuinticWaypoints.add(new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(90)));
                interiorQuinticWaypoints.add(new Pose2d(2,0, new Rotation2d(0)));
                //     interiorQuinticWaypoints.add(new Pose2d(1, 0,new Rotation2d(0)));
        // An example trajectory to follow.  All units in meters.
      /*   Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(.5, .5), new Translation2d(1, 1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                config);*/
           //     Trajectory exampleTrajectory =
          //  TrajectoryGenerator.generateTrajectory(
         //     interiorQuinticWaypoints,
          //      config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


                return new SequentialCommandGroup(
                
           new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(s_Vision.CameraGet(), s_Swerve.getYaw()))),
            swerveControllerCommand
        );
    }
}