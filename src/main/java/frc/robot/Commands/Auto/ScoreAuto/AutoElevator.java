package frc.robot.Commands.Auto.ScoreAuto;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ScoreSystem.ElevatorSubsystem;

public class AutoElevator extends Command {
    private final ElevatorSubsystem elevator;
    private final RelativeEncoder encoder;
    private final AutoAngleCoral coralCollection;
    private final double targetRotations;

    public AutoElevator(ElevatorSubsystem elevator, RelativeEncoder encoder, AutoAngleCoral coralCollection) {
        this.elevator = elevator;
        this.encoder = encoder;
        this.coralCollection = coralCollection;
        this.targetRotations = 5; 
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        encoder.setPosition(0);
        //coralCollection.initialize();
    }

    @Override
    public void execute() {
        // coralCollection.angleCoral(); 

        // if (coralCollection.isAngulated()) { 
        //     if (coralCollection.pieceOn()) {
        //         if (encoder.getPosition() < targetRotations) {
        //             elevator.powerElevator(-0.4); 
        //         } else {
        //             elevator.elevatorDisable();
        //         }
        //     } else {
        //         if (encoder.getPosition() > 0) {
        //             elevator.powerElevator(0.4); 
        //         } else {
        //             elevator.elevatorDisable(); 
        //         }
        //     }
        // } else {
        //     elevator.elevatorDisable(); 
        }
    

    @Override
    public void end(boolean interrupted) {
        elevator.elevatorDisable();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
