package frc.robot.Commands.TeleopSwerve;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ElevatorJS extends Command{
    private BooleanSupplier leftButton;
    private BooleanSupplier rightButton;
    private PS5Controller joystick;

    public ElevatorJS(BooleanSupplier leftButton, BooleanSupplier rightButton, PS5Controller joystick) {
        this.leftButton = leftButton;
        this.rightButton = rightButton;
        this.joystick = joystick;

        addRequirements(RobotContainer.elevatorEneable);
    }

}
