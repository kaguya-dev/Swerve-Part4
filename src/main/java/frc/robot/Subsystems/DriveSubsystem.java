package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
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
import frc.robot.Constants;
import frc.robot.RobotContainer;

@Logged(name = "DriveSubsystem")
public class DriveSubsystem extends SubsystemBase {
    public SwerveModule[] swerveMods;

    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swervePoser;
    public ChassisSpeeds swerveChassis;

    public DriveSubsystem() {
        swerveMods = new SwerveModule[] {
                new SwerveModule(Constants.SwerveModulesContants.MOD0),
                new SwerveModule(Constants.SwerveModulesContants.MOD1),
                new SwerveModule(Constants.SwerveModulesContants.MOD2),
                new SwerveModule(Constants.SwerveModulesContants.MOD3) };

        swerveMods[1].isRight(true);
        swerveMods[3].isRight(true);

        swerveChassis = new ChassisSpeeds();
        swerveOdometry = new SwerveDriveOdometry(Constants.kDriveKinematics, RobotContainer.getGyroAngleAsR2D(),
                getPositions());

        swervePoser = new SwerveDrivePoseEstimator(Constants.kDriveKinematics,
                RobotContainer.getGyroAngleAsR2D(), getPositions(), new Pose2d());

        // All other subsystem initialization
        // ...

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> drive(swerveChassis.vxMetersPerSecond, swerveChassis.vyMetersPerSecond,
                            swerveChassis.omegaRadiansPerSecond, false), // Method that will drive the robot given ROBOT
                    // RELATIVE ChassisSpeeds. Also optionally outputs
                    // individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for
                                                    // holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

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

    public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s,
            boolean fieldcentric) {
        SwerveModuleState[] swerveModuleStates;

        swerveChassis.vxMetersPerSecond = xVelocity_m_per_s;
        swerveChassis.vyMetersPerSecond = yVelocity_m_per_s;
        swerveChassis.omegaRadiansPerSecond = omega_rad_per_s;

        if (Math.abs(xVelocity_m_per_s) < Constants.JOY_DEADBAND)
            xVelocity_m_per_s = 0;

        if (Math.abs(yVelocity_m_per_s) < Constants.JOY_DEADBAND)
            yVelocity_m_per_s = 0;

        if (Math.abs(omega_rad_per_s) < Constants.JOY_DEADBAND)
            omega_rad_per_s = 0;

        if (fieldcentric) { // field-centric swerve
            swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelocity_m_per_s,
                            yVelocity_m_per_s,
                            omega_rad_per_s,
                            Rotation2d.fromDegrees(RobotContainer.getGyroAngleAsR2D().getDegrees())));
        } else { // robot-centric swerve; does not use IMU
            swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(swerveChassis);
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED);

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Actual State Angle mod n"
                    .concat(String.valueOf(mod.getModuleNumber())), mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Actual State Speed mod n"
                    .concat(String.valueOf(mod.getModuleNumber())), mod.getState().speedMetersPerSecond);

            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()]);
        }
    }

    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < swerveMods.length; i++) {
            positions[i] = swerveMods[i].getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        System.out.println(pose);
        swerveOdometry.resetPosition(RobotContainer.getGyroAngleAsR2D(), getPositions(), pose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return swerveChassis;
    }

}