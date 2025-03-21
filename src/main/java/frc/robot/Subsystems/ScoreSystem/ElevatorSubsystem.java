package frc.robot.Subsystems.ScoreSystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private double[] lHeights = {50,100,150,200};
    private double actualPosLimit = 0;
    private RelativeEncoder elevatorEncoder;
    private DigitalInput calibrationSwitch;
    private boolean zeroPoint;
    private SparkMaxConfig leftMotorConfig;
    private SparkMaxConfig rightMotorConfig;
    private PIDController elevatorPIDController;

    public ElevatorSubsystem() {
        elevatorPIDController = new PIDController(Constants.kElevatorKP, Constants.kElevatorKI, Constants.kElevatorKD);

        leftMotor = new SparkMax(Constants.kLeftElevatorMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.kRightElevatorMotorID, MotorType.kBrushless);

        leftMotorConfig = new SparkMaxConfig();
        rightMotorConfig = new SparkMaxConfig();

        leftMotorConfig.inverted(false); 
        leftMotorConfig.idleMode(IdleMode.kBrake); 

        rightMotorConfig.inverted(true); 
        rightMotorConfig.idleMode(IdleMode.kBrake); 

        elevatorEncoder = leftMotor.getEncoder();
        calibrationSwitch = new DigitalInput(9);
        zeroPoint = true;
        
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        

        Shuffleboard.getTab("ScoreSystem").addBoolean("CalibrationSwitch", () -> calibrationSwitch.get());
        Shuffleboard.getTab("ScoreSystem").addDouble("ElevatorPosition", () -> elevatorEncoder.getPosition());
    }

    public void powerElevator(double power) {
        //power = MathUtil.clamp(power, -0.65, 0.65);
        SmartDashboard.putNumber("Elevator Power", power);

        
        if (!zeroPoint) {
            leftMotor.set(power);
            rightMotor.set(power);
        } else {
            //leftMotor.set(MathUtil.clamp(power, -1, 0));
            //rightMotor.set(MathUtil.clamp(power, -1, 0));
            leftMotor.set(power);
            rightMotor.set(power);
        }
    }

    public void elevatorDisable() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    /**
     * Moves the elevator to a specific position using the PID controller.
     *
     * @param position The target position for the elevator.
     */
    public void setElevator(double position) {
        double output = elevatorPIDController.calculate(elevatorEncoder.getPosition(), position);

        if (!zeroPoint) {
            powerElevator(output); 
        } else {
            MathUtil.clamp(output, 0, Constants.kElevatorSpeedClamper);
        }
    }

    private void lowCalibratePID() {
        elevatorEncoder.setPosition(0);
    }

    public void setLPos(int posID){
        switch (posID) {
            //L1
            case 0:
                actualPosLimit = lHeights[0];
                break;
            //L2
            case 1:
                actualPosLimit = lHeights[1];
                break;
            //L3
            case 2: 
                actualPosLimit = lHeights[2];
                break;
            //Collect Point
            case 3:
                actualPosLimit = lHeights[3];
                break;
        }
    }

    @Override
    public void periodic() {
        if (calibrationSwitch.get()) {
            //lowCalibratePID();
        }

        if(-elevatorEncoder.getPosition() > 0){
            zeroPoint = false;
        }else{
            zeroPoint = true;
        }

    SmartDashboard.putNumber("Elevator Encoder Position", -elevatorEncoder.getPosition());
    SmartDashboard.putNumber("ActualLimitPos", actualPosLimit);
    SmartDashboard.putBoolean("ZeroPointed Elevator", zeroPoint);
    }
}