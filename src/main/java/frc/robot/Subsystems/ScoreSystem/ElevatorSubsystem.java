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
    private RelativeEncoder elevatorEncoder;
    private DigitalInput calibrationSwitch;
    private boolean isAtMaxHeight;
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
        calibrationSwitch = new DigitalInput(Constants.kMicroSwitchPWMPort);
        isAtMaxHeight = false;
        
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Shuffleboard.getTab("ScoreSystem").addBoolean("CalibrationSwitch", () -> calibrationSwitch.get());
        Shuffleboard.getTab("ScoreSystem").addDouble("ElevatorPosition", () -> elevatorEncoder.getPosition());
    }

    public void powerElevator(double power) {
        power = MathUtil.clamp(power, -0.5, 0.5);
        SmartDashboard.putNumber("Elevator Power", power);

        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void elevatorDisable() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public void setElevator(double position) {
        double output = elevatorPIDController.calculate(elevatorEncoder.getPosition(), position);

        if (position < Constants.kMaxHeight) {
            isAtMaxHeight = false; 
            powerElevator(output); 
        } else {
            isAtMaxHeight = true; 
        }
    }

    private void lowCalibratePID() {
        elevatorEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        if (calibrationSwitch.get()) {
            lowCalibratePID();
        }
    }
}