package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{

    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private RelativeEncoder encoder;

    private SparkMaxConfig leftConfig;
    private SparkMaxConfig rightConfig;
    
    public ElevatorSubsystem(){
        leftMotor = new SparkMax(Constants.leftElevatorMotor, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.rightElevatorMotor, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        leftConfig.inverted(false);
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.inverted(true);
        rightConfig.idleMode(IdleMode.kBrake);

        encoder = leftMotor.getEncoder();
        
    }
}
