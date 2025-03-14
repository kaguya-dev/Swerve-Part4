package frc.robot.Subsystems.SwerveDrive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
    public SwerveModule[] swerveMods;

    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swervePoser;

    public DriveSubsystem() {
        swerveMods = new SwerveModule[] {
            new SwerveModule(Constants.SwerveModuleConstants.MOD0),
            new SwerveModule(Constants.SwerveModuleConstants.MOD1),
            new SwerveModule(Constants.SwerveModuleConstants.MOD2),
            new SwerveModule(Constants.SwerveModuleConstants.MOD3) 
        };
        
        swerveMods[1].isRight(false);
        swerveMods[3].isRight(false);
            
        swerveOdometry = new SwerveDriveOdometry(Constants.kDriveKinematics, RobotContainer.getGyroAngleAsR2D(),
                getPositions());

        swervePoser = new SwerveDrivePoseEstimator(Constants.kDriveKinematics,
                RobotContainer.getGyroAngleAsR2D(), getPositions(), new Pose2d());
    }

    public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s, boolean fieldcentric) {
        SwerveModuleState[] swerveModuleStates;

        if (Math.abs(xVelocity_m_per_s) < Constants.kControllerDeadband)
            xVelocity_m_per_s = 0;

        if (Math.abs(yVelocity_m_per_s) < Constants.kControllerDeadband)
            yVelocity_m_per_s = 0;

        if (Math.abs(omega_rad_per_s) < Constants.kControllerDeadband)
            omega_rad_per_s = 0;

        if (fieldcentric) { // field-centric swerve
            swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelocity_m_per_s,
                            yVelocity_m_per_s,
                            omega_rad_per_s,
                            Rotation2d.fromDegrees(RobotContainer.getGyroAngleAsR2D().getDegrees())));
        } else { // robot-centric swerve; does not use IMU
            swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
                    new ChassisSpeeds(
                            xVelocity_m_per_s,
                            yVelocity_m_per_s,
                            omega_rad_per_s));
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Actual State Angle mod n"
                    .concat(String.valueOf(mod.getModuleNumber())), mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Actual State Speed mod n"
                    .concat(String.valueOf(mod.getModuleNumber())), mod.getState().speedMetersPerSecond);

            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()]);
        }
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < swerveMods.length; i++) {
            positions[i] = swerveMods[i].getPosition();
        }
        return positions;
    }

   
}