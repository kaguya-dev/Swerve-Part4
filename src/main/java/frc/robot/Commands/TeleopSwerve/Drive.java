package frc.robot.Commands.TeleopSwerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Utils.Constants;

public class Drive extends Command {

    private final DoubleSupplier mVxSupplier;
    private final DoubleSupplier mVySupplier;
    private final DoubleSupplier mOmegaSupplier;
    private final BooleanSupplier mFieldCentrSupplier;

    public Drive(DoubleSupplier mVxSupplier, DoubleSupplier mVySupplier, DoubleSupplier mOmegaSupplier,
            BooleanSupplier mFieldCentrSupplier) {
        addRequirements(RobotContainer.driver);
        this.mVxSupplier = mVxSupplier;
        this.mVySupplier = mVySupplier;
        this.mOmegaSupplier = mOmegaSupplier;
        this.mFieldCentrSupplier = mFieldCentrSupplier;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void execute() {
        double xInput = mVxSupplier.getAsDouble();
        double yInput = mVySupplier.getAsDouble();
        double omegaInput = mOmegaSupplier.getAsDouble();

        RobotContainer.driver.drive(
                xInput * Constants.MAX_SPEED,
                yInput * Constants.MAX_SPEED,
                omegaInput * Constants.MAX_ANGULAR_SPEED,
                mFieldCentrSupplier.getAsBoolean());
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}