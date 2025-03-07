// Necessary imports for the code to function
package frc.robot.Commands.Auto;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Utils.Constants;

/**
 * Command for automatically aligning the robot to a target angle using a PID controller.
 * This command uses the Limelight's X value to calculate the necessary rotation to align with the target.
 */
public class AutoAngleAlignAT extends Command {
    // PID controller for angle alignment
    //private final PIDController anglePIDController;

    // Supplier for the Limelight's X value (horizontal offset from the target)
    //private final DoubleSupplier limeXSupplier;


    /**
     * Constructor for the AutoAngleAlignAT command.
     *
     * @param targetID The ID of the target to align with.
     */
     
    /*public AutoAngleAlignAT(int targetID) {

        // Initialize the Limelight X value supplier
        //limeXSupplier = () -> RobotContainer.limelight.getLimeXValue();

        // Initialize the PID controller with constants for angle alignment
        anglePIDController = new PIDController(
            Constants.kAngleAlignKP, 
            Constants.kAngleAlignKI, 
            Constants.kAngleAlignKD
        );

        anglePIDController.setIZone(0.1); 
    }*/

    /**
     * Method called periodically while the command is active.
     * This calculates the necessary rotation to align with the target and applies it to the drive subsystem.
     */
    @Override
    public void execute() {
        // Calculate the rotation output using the PID controller and Limelight X value
        //double rotationOutput = anglePIDController.calculate(limeXSupplier.getAsDouble());

        // Drive the robot with no forward/strafe movement, only rotation
        //RobotContainer.driver.drive(0, 0, rotationOutput, false);
    }

    /**
     * Method called when the command ends, either due to interruption or completion.
     *
     * @param interrupted Indicates whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {

    }

    /**
     * Method to check if the command is finished.
     *
     * @return Always returns false, as this command is intended to run continuously.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}