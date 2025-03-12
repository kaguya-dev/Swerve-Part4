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
    private SparkMax angulationCoral;
    private VictorSPX coralIntake;

    //PID Controller
    private PIDController coralIntakePID;

    //Encoders
    private RelativeEncoder angulationEncoder;

    //Configurators
    private SparkMaxConfig angulationCoralConfig;

    public IntakeSubsystem(){
        angulationCoral = new SparkMax(Constants.kIntakeAngularCoralMotorID, MotorType.kBrushless);
        angulationCoralConfig = new SparkMaxConfig();
        coralIntake = new VictorSPX(Constants.kIntakeCoralMotorID);
        coralIntakePID = new PIDController(Constants.kCoralIntakeKP, Constants.kCoralIntakeKI, Constants.kCoralIntakeKD);
        coralIntakePID.enableContinuousInput(0, 0.02);

        angulationCoralConfig.idleMode(IdleMode.kBrake);
        angulationCoral.configure(angulationCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        angulationEncoder = angulationCoral.getEncoder();

    }


    /**
     * @param position put in degrees
     */
    public void setCoralAngularPosition(double position){
        double power = coralIntakePID.calculate(angulationEncoder.getPosition(), position);
        angulationCoralSetPower(power);
    }

    public void angulationCoralSetPower(double power){
        angulationCoral.set(power);
    }

    public void coralDisable(){
        coralIntake.set(ControlMode.PercentOutput, 0);
    }

    public void coralIntake(double power){
        coralIntake.set(ControlMode.PercentOutput, power);
    }

    // public void algaeIntake(double power, boolean input){
    //     if(input){
    //         algaeIntakeLeft.set(power);
    //         algaeIntakeRight.set(power);
    //     }else{
    //         algaeIntakeLeft.set(power);
    //         algaeIntakeRight.set(power);
    //     }
    // }

    // public void algaeIntakeDisable(){
    //     algaeIntakeLeft.set(0);
    //     algaeIntakeRight.set(0);
    // }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Coral Angulation Value", angulationEncoder.getPosition());
    }

}
