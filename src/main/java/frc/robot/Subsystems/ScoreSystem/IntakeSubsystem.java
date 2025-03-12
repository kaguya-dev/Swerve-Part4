package frc.robot.Subsystems.ScoreSystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;

public class IntakeSubsystem extends SubsystemBase{

    //Motors
    private SparkMax angulationCoralMotor;
    private VictorSPX coralIntake;
    
    //PID Controller
    private PIDController coralIntakePID;

    //Encoders
    
    private RelativeEncoder coralAngulationEncoder;

    //Configurators
    private SparkMaxConfig angulationCoralConfig;

    public IntakeSubsystem(){
        angulationCoralMotor = new SparkMax(Constants.kIntakeAngularCoralMotorID, MotorType.kBrushless);
        angulationCoralConfig = new SparkMaxConfig();
        coralIntake = new VictorSPX(Constants.kIntakeCoralMotorID);
        

        coralIntakePID = new PIDController(Constants.kCoralIntakeKP, Constants.kCoralIntakeKI, Constants.kCoralIntakeKD);
        coralIntakePID.enableContinuousInput(0, 0.02);

        //algaeAngulationEncoder = angulationCoralMotor.getAlternateEncoder();
        //angulationCoralConfig.alternateEncoder.countsPerRevolution(360);
        angulationCoralConfig.idleMode(IdleMode.kBrake);
        angulationCoralMotor.configure(angulationCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //algaeAngulationEncoder = algaeIntakeRight.getAlternateEncoder();
        coralAngulationEncoder = angulationCoralMotor.getEncoder();
    }

    /**
     * @param position put in degrees
     */
    public void setCoralAngularPosition(double position){
        double power = coralIntakePID.calculate(coralAngulationEncoder.getPosition(), position);
        angulationCoralSetPower(power);
    }

    public void angulationCoralSetPower(double power){
        angulationCoralMotor.set(power);
    }

    // public void algaeAngulation(double power){
    //     algaeAngulation1.set(ControlMode.PercentOutput, power);
    // }

    public void coralDisable(){
        coralIntake.set(ControlMode.PercentOutput, 0);
    }

    public void coralIntake(double power){
        coralIntake.set(ControlMode.PercentOutput, power);
    }

    public void controlCoralAngulation(double yAxisValue) {
        if (yAxisValue > Constants.kControllerDeadband) {
            angulationCoralSetPower(0.15);
        } else if (yAxisValue < -Constants.kControllerDeadband) {
            angulationCoralSetPower(-0.15);
        } else {
            angulationCoralSetPower(0);
        }
    }
   
    @Override
    public void periodic(){
        //SmartDashboard.putNumber("Algae Angulation Value", algaeAngulationEncoder.getPosition());
        SmartDashboard.putNumber("Coral Angulation Value", coralAngulationEncoder.getPosition());
    }

}