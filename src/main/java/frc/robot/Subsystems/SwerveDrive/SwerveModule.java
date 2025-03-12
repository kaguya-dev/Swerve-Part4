package frc.robot.Subsystems.SwerveDrive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Constants.SwerveModuleConstants;


public class SwerveModule extends SubsystemBase{
    private SparkMax turnMotor;
    private SparkMax speedMotor;
    private SwerveModuleState moduleState;
    private CANcoder absoluteEncoder;
    private RelativeEncoder speedEncoder;
    private PIDController pidController;
    private boolean isRight;
    private int moduleNumber;

    public SwerveModule(SwerveModuleConstants moduleID){
        this.turnMotor = new SparkMax(moduleID.getAngleMotorID(), MotorType.kBrushless);
        this.speedMotor = new SparkMax(moduleID.getDriveMotorID(), MotorType.kBrushless);
        this.absoluteEncoder = new CANcoder(moduleID.getCancoderID());
        this.speedEncoder = speedMotor.getEncoder();
        this.moduleNumber = moduleID.getModuleNumber();
        this.moduleState = new SwerveModuleState();
        updateState();
        this.pidController = new PIDController(Constants.kSwerveAngleKP, Constants.kSwerveAngleKI, Constants.kSwerveAngleKD);
        pidController.setIZone(5);
        pidController.setTolerance(1);
    }

    @Override
    public void periodic(){
        updateState();
    }

    private void updateState(){
        moduleState.angle = getAngleInR2D();
        double stateSpeed = (speedEncoder.getVelocity() * Constants.kWheelCircumferenceMeters) / 60;

        moduleState.speedMetersPerSecond = stateSpeed;

        SmartDashboard.putNumber("Speed mod n".concat(String.valueOf(this.moduleNumber)), stateSpeed);
    }

    public SwerveModuleState getState(){
        updateState();
        return moduleState;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(speedEncoder.getPosition(), getAngleInR2D());
    }

    private Rotation2d getAngleInR2D(){
        SmartDashboard.putNumber("Angle mod n".concat(String.valueOf(moduleNumber)), 
            absoluteEncoder.getAbsolutePosition().getValue().magnitude() * 360);
        return new Rotation2d(absoluteEncoder.getAbsolutePosition().getValue());
    }

    public void setSpeedPower(double power){
        speedMotor.set(power);
    }

    public void setTurnSpeed(double power){
        turnMotor.set(power);
    }

    public void isRight(boolean right){
        isRight = right;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize( getState().angle);

        double speedPower = (desiredState.speedMetersPerSecond / Constants.kMaxPower) * (isRight? -1 : 1);
        SmartDashboard.putNumber("SET PRE-Speed mod n".concat(String.valueOf(moduleNumber)), speedPower);
        setSpeedPower(MathUtil.clamp(speedPower, -Constants.kMaxPower, Constants.kMaxPower));
        SmartDashboard.putNumber("SET CLAMPED-speed mod n".concat(String.valueOf(moduleNumber)), speedPower);

        double setpoint = desiredState.angle.getDegrees();
        double measure = getState().angle.getDegrees();

        double error = setpoint - measure;
        if (Math.abs(error) > 180) {
            if (error > 0) {
                setpoint -= 360;
            } else {
                setpoint += 360;
            }
        }

        double pidValue = pidController.calculate(measure, setpoint);

        SmartDashboard.putNumber("SET Angle mod n".concat(String.valueOf(moduleNumber)), pidValue);
        setTurnSpeed(pidValue);
    }

    public int getModuleNumber(){
        return moduleNumber;
    }
}