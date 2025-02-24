// Necessary imports for the code to function
package frc.robot.Commands.TeleopSwerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Utils.Constants;

/**
 * Class responsible for implementing the robot's drive command during teleoperated mode.
 * This command uses suppliers to obtain velocity and orientation inputs,
 * and applies these inputs to the robot's drive subsystem.
 */
public class Drive extends Command {

    // Suppliers for forward velocity, strafe velocity, and rotational velocity
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier rotSpeedSupplier;

    // Supplier for determining if the control is field-centric
    private final BooleanSupplier mFieldCentricSupplier;

    /**
     * Constructor for the Drive class.
     *
     * @param xSpeedSupplier Supplier for forward/backward velocity.
     * @param ySpeedSupplier Supplier for left/right velocity.
     * @param rotSpeedSupplier Supplier for rotational velocity.
     * @param mFieldCentricSupplier Boolean supplier to determine if the control is field-centric.
     */
    public Drive(DoubleSupplier xSpeedSupplier,DoubleSupplier ySpeedSupplier,
    DoubleSupplier rotSpeedSupplier,BooleanSupplier mFieldCentricSupplier) {
        // Adds the drive subsystem as a requirement for this command
        addRequirements(RobotContainer.driver);

        // Initializes the suppliers
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;
        this.mFieldCentricSupplier = mFieldCentricSupplier;
    }

    /**
     * Method called when the command ends, either due to interruption or completion.
     *
     * @param interrupted Indicates whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    /**
     * Method called periodically while the command is active.
     * Here, the inputs from the suppliers are processed and applied to the drive subsystem.
     */
    @Override
    public void execute() {
        // Gets inputs from the suppliers
        double xInput = xSpeedSupplier.getAsDouble();
        double yInput = ySpeedSupplier.getAsDouble();
        double rotInput = rotSpeedSupplier.getAsDouble();

        // Applies the inputs to the drive subsystem, scaling them by the speed constants
        RobotContainer.driver.drive(
                xInput * Constants.kMaxSpeed,
                yInput * Constants.kMaxSpeed,
                rotInput * Constants.kMaxAngularSpeed,
                mFieldCentricSupplier.getAsBoolean());
    }

    /**
     * Method called when the command is initialized.
     */
    @Override
    public void initialize() {
        super.initialize();
    }
}