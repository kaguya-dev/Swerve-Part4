package frc.robot.Commands.Auto.ScoreAuto;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ScoreSystem.ElevatorSubsystem;

public class AutoElevator extends Command{
    DoubleSupplier encoderRotation;
    double finalPosition;
    ElevatorSubsystem elevator;
    RelativeEncoder encoder;

@Override
    public void initialize() {
        super.initialize();

    }

    @Override
    public void execute() {
        

    }

    @Override
    public void end(boolean interrupted) {
        
    }
    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
