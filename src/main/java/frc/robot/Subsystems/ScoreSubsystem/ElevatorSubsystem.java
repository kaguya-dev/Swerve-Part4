package frc.robot.Subsystems.ScoreSubsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;

public class ElevatorSubsystem extends SubsystemBase{

    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private RelativeEncoder encoder;
    private DigitalInput calibrationSwitch;
    private boolean maxHeight;

    private SparkMaxConfig leftConfig;
    private SparkMaxConfig rightConfig;
    
    private PIDController pidController;

    public ElevatorSubsystem(){
        pidController = new PIDController(Constants.elevatorKP, Constants.elevatorKI, Constants.elevatorKD);

        leftMotor = new SparkMax(Constants.leftElevatorMotor, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.rightElevatorMotor, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        leftConfig.inverted(false);
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.inverted(true);
        rightConfig.idleMode(IdleMode.kBrake);

        encoder = leftMotor.getEncoder();
        calibrationSwitch = new DigitalInput(Constants.microSwitchPWMPort);
        maxHeight = false;
        
        Shuffleboard.getTab("ScoreSystem").addBoolean("CalibrationSwitch", (() -> calibrationSwitch.get()));
        Shuffleboard.getTab("ScoreSystem").addDouble("ElevatorPosition", (() -> encoder.getPosition()));
    }

    private void powerElevator(double power){
        //Speed clamper a definir
        power = MathUtil.clamp(power, -Constants.elevatorSpeedClamper, Constants.elevatorSpeedClamper);
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void setElevator(double position){
        double calculate = pidController.calculate(encoder.getPosition(), position);
        if(position < Constants.MAXHEIGHT){
            maxHeight = false;
            powerElevator(calculate);
        }else{
            maxHeight = true;
        }
    }

    private void lowCalibratePID(){
        encoder.setPosition(0);
    }

    @Override
    public void periodic(){
        if(calibrationSwitch.get())
            lowCalibratePID();
        Constants.elevatorMaxHeight.set(maxHeight);
    }
}
