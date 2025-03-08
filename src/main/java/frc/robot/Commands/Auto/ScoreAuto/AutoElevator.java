package frc.robot.Commands.Auto.ScoreAuto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ScoreSystem.ElevatorSubsystem;

public class AutoElevator extends Command{
    DoubleSupplier encoderRotation;
    double finalPosition;
    ElevatorSubsystem elevator;
}
