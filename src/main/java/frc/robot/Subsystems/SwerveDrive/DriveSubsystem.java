// Necessary imports for the code to function
package frc.robot.Subsystems.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Utils.Constants;

/**
 * Subsystem responsible for controlling the swerve drive.
 * This subsystem manages the swerve modules, odometry, and path following using PathPlanner.
 */
public class DriveSubsystem extends SubsystemBase {

    // Array of swerve modules
    public SwerveModule[] swerveModules;

    // Odometry and pose estimator for tracking robot position
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swervePoseEstimator;

    // Chassis speeds for robot movement
    public ChassisSpeeds swerveChassisSpeeds;

    /**
     * Constructor for the DriveSubsystem.
     * Initializes swerve modules, odometry, pose estimator, and configures AutoBuilder for path following.
     */
    public DriveSubsystem() {
        // Initialize swerve modules with constants from Constants class
        swerveModules = new SwerveModule[] {
                new SwerveModule(Constants.SwerveModulesContants.kMOD0),
                new SwerveModule(Constants.SwerveModulesContants.kMOD1),
                new SwerveModule(Constants.SwerveModulesContants.kMOD2),
                new SwerveModule(Constants.SwerveModulesContants.kMOD3)
        };

        // Set the right-side modules as right-facing
        swerveModules[1].isRight(true);
        swerveModules[3].isRight(true);

        // Initialize chassis speeds
        swerveChassisSpeeds = new ChassisSpeeds();

        // Initialize odometry with the current gyro angle and module positions
        swerveOdometry = new SwerveDriveOdometry(Constants.kDriveKinematics, RobotContainer.getGyroAngleAsR2D(), getModulePositions());

        // Initialize pose estimator with kinematics, gyro angle, module positions, and initial pose
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.kDriveKinematics, RobotContainer.getGyroAngleAsR2D(), getModulePositions(), new Pose2d());

        // Load the RobotConfig from the GUI settings (typically stored in Constants)
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder for path following
            AutoBuilder.configure(
                    this::getPose, 
                    this::resetPose, 
                    this::getCurrentSpeeds,
                    (speeds, feedforwards) -> drive(swerveChassisSpeeds.vxMetersPerSecond, swerveChassisSpeeds.vyMetersPerSecond, swerveChassisSpeeds.omegaRadiansPerSecond, false), // Method to drive the robot using robot-relative ChassisSpeeds
                    new PPHolonomicDriveController( // Built-in path following controller for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // Robot configuration
                    () -> { // Boolean supplier to mirror the path for the red alliance
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    /**
     * Drives the robot using the specified velocities and field-centric mode.
     *
     * @param xVelocity_m_per_s The velocity in the X direction (meters per second).
     * @param yVelocity_m_per_s The velocity in the Y direction (meters per second).
     * @param rotationSpeed_rad_per_s The rotational velocity (radians per second).
     * @param fieldCentric Whether to use field-centric driving.
     */
    public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double rotationSpeed_rad_per_s, boolean fieldCentric) {
        SwerveModuleState[] swerveModuleStates;

        // Update chassis speeds
        swerveChassisSpeeds.vxMetersPerSecond = xVelocity_m_per_s;
        swerveChassisSpeeds.vyMetersPerSecond = yVelocity_m_per_s;
        swerveChassisSpeeds.omegaRadiansPerSecond = rotationSpeed_rad_per_s;

        // Apply deadband to velocities
        if (Math.abs(xVelocity_m_per_s) < Constants.kControllerDeadband) xVelocity_m_per_s = 0;
        if (Math.abs(yVelocity_m_per_s) < Constants.kControllerDeadband) yVelocity_m_per_s = 0;
        if (Math.abs(rotationSpeed_rad_per_s) < Constants.kControllerDeadband) rotationSpeed_rad_per_s = 0;

        // Calculate swerve module states based on field-centric or robot-centric mode
        if (fieldCentric) {
            swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelocity_m_per_s,
                            yVelocity_m_per_s,
                            rotationSpeed_rad_per_s,
                            Rotation2d.fromDegrees(RobotContainer.getGyroAngleAsR2D().getDegrees())
                    )
            );
        } else {
            swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(swerveChassisSpeeds);
        }

        // Desaturate wheel speeds to ensure they are within the maximum speed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

        // Set desired states for each swerve module
        for (SwerveModule module : swerveModules) {
            SmartDashboard.putNumber("Actual State Angle mod " + module.getModuleNumber(), module.getState().angle.getDegrees());
            SmartDashboard.putNumber("Actual State Speed mod " + module.getModuleNumber(), module.getState().speedMetersPerSecond);
            module.setDesiredState(swerveModuleStates[module.getModuleNumber()]);
        }
    }

    /**
     * Returns the positions of all swerve modules.
     *
     * @return An array of SwerveModulePosition objects representing the positions of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    /**
     * Returns the current pose of the robot.
     *
     * @return The current pose as a Pose2d object.
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Resets the robot's pose to the specified position.
     *
     * @param pose The new pose to reset to.
     */
    public void resetPose(Pose2d pose) {
        System.out.println(pose);
        swerveOdometry.resetPosition(RobotContainer.getGyroAngleAsR2D(), getModulePositions(), pose);
    }

    /**
     * Returns the current chassis speeds of the robot.
     *
     * @return The current chassis speeds as a ChassisSpeeds object.
     */
    public ChassisSpeeds getCurrentSpeeds() {
        return swerveChassisSpeeds;
    }
}