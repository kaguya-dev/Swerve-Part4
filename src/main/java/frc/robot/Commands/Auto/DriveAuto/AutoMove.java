package frc.robot.Commands.Auto.DriveAuto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

public class AutoMove extends Command{
    Translation2d deltaPose;
    Translation2d finalPose;
    DriveSubsystem swerve;

    public AutoMove(Translation2d deltaPose,Translation2d finalPose, DriveSubsystem swerve) {
        this.deltaPose = deltaPose;
        this.finalPose = finalPose;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
    
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    
}
