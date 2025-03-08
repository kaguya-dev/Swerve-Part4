package frc.robot.Commands.Auto.DriveAuto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

public class AutoAngleAlign extends Command{
    DoubleSupplier angSupplier;
    Rotation2d finalAngle;
    DriveSubsystem swerve;

    public AutoAngleAlign(DoubleSupplier angSupplier, Rotation2d finalAngle, DriveSubsystem swerve) {
        this.angSupplier = angSupplier;
        this.finalAngle = finalAngle;
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
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
