package frc.robot.Commands.ScoreCommand;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ElevatorJS extends Command {
    private BooleanSupplier leftButton;
    private BooleanSupplier rightButton;

    public ElevatorJS(BooleanSupplier leftButton, BooleanSupplier rightButton) {
        this.leftButton = leftButton;
        this.rightButton = rightButton;

        addRequirements(RobotContainer.elevatorEneable);
    }

    @Override 
    public void execute() {
        if (leftButton.getAsBoolean()) {
            RobotContainer.elevatorEneable.powerElevator(0.5);
        } else if (rightButton.getAsBoolean()) {
            RobotContainer.elevatorEneable.powerElevator(-0.5);
        } else {
            RobotContainer.elevatorEneable.powerElevator(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.elevatorEneable.powerElevator(0);
    }
}