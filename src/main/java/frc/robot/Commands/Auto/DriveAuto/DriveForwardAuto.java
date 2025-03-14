package frc.robot.Commands.Auto.DriveAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Utils.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;

public class DriveForwardAuto extends Command {
    private final double spd = 0.3; 
    private final double time = 1.5; 
    private double startTime;
    private Pose2d initialPose;

    public DriveForwardAuto() {
        addRequirements(RobotContainer.driver);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        initialPose = RobotContainer.driver.swervePoser.getEstimatedPosition(); 
       
    }

    @Override
    public void execute() {
        RobotContainer.driver.drive(0, spd * Constants.kMaxSpeed, 0, false);
        
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) >= time;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.driver.drive(0, 0, 0, false); // Para o robô

        Pose2d finalPose = RobotContainer.driver.swervePoser.getEstimatedPosition();
        double distanceMoved = finalPose.getTranslation().getDistance(initialPose.getTranslation());

        double expectedDistance = spd * Constants.kMaxSpeed * time;
        double error = Math.abs(distanceMoved - expectedDistance);
        SmartDashboard.putNumber("Error", error);
    }
}