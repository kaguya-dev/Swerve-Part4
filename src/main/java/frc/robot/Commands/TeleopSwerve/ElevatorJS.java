package frc.robot.Commands.TeleopSwerve;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ElevatorJS extends Command{
    private BooleanSupplier leftButton;
    private BooleanSupplier rightButton;
    private PS5Controller joystick;

    public ElevatorJS(PS5Controller joystick) {
        this.joystick = joystick;
        leftButton = () -> joystick.getL1ButtonPressed();
        rightButton = () -> joystick.getR1ButtonPressed();

        addRequirements(RobotContainer.elevatorEneable);
    }

}
