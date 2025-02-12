package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveModulesContants;

@Logged(name = "SwerveModule")
public class SwerveModule extends SubsystemBase{
    private SparkMax turnMotor;
    private SparkMax speedMotor;
    private SwerveModuleState moduleState;
    private CANcoder absoluteEncoder;
    private RelativeEncoder speedEncoder;
    private PIDController pidController;
    private int moduleNumber;

    public SwerveModule(SwerveModulesContants moduleID){
        this.turnMotor = new SparkMax(moduleID.getAngleMotorID(), MotorType.kBrushless);
        this.speedMotor = new SparkMax(moduleID.getDriveMotorID(), MotorType.kBrushless);
        this.absoluteEncoder = new CANcoder(moduleID.getCancoderID());
        this.speedEncoder = speedMotor.getAlternateEncoder();
        this.moduleNumber = moduleID.getModuleNumber();
        this.moduleState = new SwerveModuleState();
        updateState();
        pidController = RobotContainer.pid;
    }

    @Override
    public void periodic(){
        updateState();
    }

    private void updateState(){

        moduleState.angle = getAngleInR2D();
        moduleState.speedMetersPerSecond = (speedEncoder.getVelocity() * Constants.kWheelCircuferenceMeters)/60;
    }

    public SwerveModuleState getState(){
        updateState();
        return moduleState;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(speedEncoder.getPosition(), getAngleInR2D());
    }

    private Rotation2d getAngleInR2D(){
        //Rotation2d angleOffset = new Rotation2d(90);
        SmartDashboard.putNumber("Angle mod n".concat(String.valueOf(moduleNumber)), 
            absoluteEncoder.getAbsolutePosition().getValue().magnitude() *360);
        return new Rotation2d(absoluteEncoder.getAbsolutePosition().getValue());
    }

    public void setSpeedPower(double power){
        speedMotor.set(power);
    }

    public void setTurnSpeed(double power){
        turnMotor.set(power);
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        desiredState.optimize(getState().angle);

        double speedPower = desiredState.speedMetersPerSecond / Constants.MAX_SPEED;

        if(speedPower >= 0)
            setSpeedPower(MathUtil.clamp(speedPower, 0, 0.20));
        else    
            setSpeedPower(MathUtil.clamp(speedPower, -0.20, 0));
        setTurnPosition(transform(desiredState.angle.getDegrees()), transform(getState().angle.getDegrees())); // Rotation2d angle does not
                                                                                         // give degrees
    }

    public void setTurnPosition(double setpoint, double measure){
        SmartDashboard.putNumber("SET Angle mod n".concat(String.valueOf(moduleNumber)), 
        pidController.calculate(measure, setpoint));
        turnMotor.set(pidController.calculate(measure, setpoint));
    }

    public int getModuleNumber(){
        return moduleNumber;
    }

    public static double transform(double x) {
        double result = Math.IEEEremainder(x + 180, 360);
        return result < 0 ? result + 360 : result; // Ensure it's in [0, 359]
    }
}   
