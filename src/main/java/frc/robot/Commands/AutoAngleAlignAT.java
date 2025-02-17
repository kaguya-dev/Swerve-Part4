package frc.robot.Commands;

import java.lang.invoke.ConstantBootstraps;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Utils.Constants;

public class AutoAngleAlignAT extends Command{
    PIDController anglePID;
    DoubleSupplier limeX;
    int targetID;

    public AutoAngleAlignAT(int targetID){
        addRequirements(RobotContainer.limelight, RobotContainer.driver);
        limeX = () -> RobotContainer.limelight.getLimeXValue();
        anglePID = new PIDController(Constants.angleAlignKP, Constants.angleAlignKI, Constants.angleAlignKD);
        this.targetID = targetID;
        anglePID.setSetpoint(0);
        anglePID.setTolerance(0.04);
        anglePID.setIZone(0.1);
    }

    @Override
    public void execute(){
        RobotContainer.driver.drive(0, 0, 
            anglePID.calculate(limeX.getAsDouble()), false);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {0
        return false;
    }

}   
