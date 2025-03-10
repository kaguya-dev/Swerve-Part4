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
    private SparkMax algaeIntakeLeft;
    private SparkMax algaeIntakeRight;
    private VictorSPX algaeAngulation1;
    private VictorSPX algaeAngulation2; //Following the 1

    //PID Controller
    private PIDController coralIntakePID;

    //Encoders
    private RelativeEncoder algaeAngulationEncoder;
    private RelativeEncoder coralAngulationEncoder;

    //Configurators
    private SparkMaxConfig angulationCoralConfig;
    private SparkMaxConfig algaeLeftConfig;
    private SparkMaxConfig algaeRightConfig;

    public IntakeSubsystem(){
        angulationCoralMotor = new SparkMax(Constants.kIntakeAngularCoralMotorID, MotorType.kBrushless);
        angulationCoralConfig = new SparkMaxConfig();
        algaeLeftConfig = new SparkMaxConfig();
        algaeRightConfig = new SparkMaxConfig();
        coralIntake = new VictorSPX(Constants.kIntakeCoralMotorID);
        algaeIntakeLeft = new SparkMax(Constants.kIntakeAlgaeLeftID, MotorType.kBrushless);
        algaeIntakeRight = new SparkMax(Constants.kIntakeAlgaeRightID, MotorType.kBrushless);
        algaeAngulation1 = new VictorSPX(Constants.kAlgaeAng_1_ID);
        algaeAngulation2 = new VictorSPX(Constants.kAlgaeAng_2_ID);

        coralIntakePID = new PIDController(Constants.kCoralIntakeKP, Constants.kCoralIntakeKI, Constants.kCoralIntakeKD);
        coralIntakePID.enableContinuousInput(0, 0.02);

        //algaeAngulationEncoder = angulationCoralMotor.getAlternateEncoder();
        //angulationCoralConfig.alternateEncoder.countsPerRevolution(360);
        angulationCoralConfig.idleMode(IdleMode.kBrake);
        angulationCoralMotor.configure(angulationCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      

        algaeLeftConfig.inverted(false);
        algaeLeftConfig.idleMode(IdleMode.kBrake);
        algaeRightConfig.inverted(true);
        algaeRightConfig.idleMode(IdleMode.kBrake);

        //algaeAngulationEncoder = algaeIntakeRight.getAlternateEncoder();
        coralAngulationEncoder = angulationCoralMotor.getEncoder();

        algaeIntakeLeft.configure(algaeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algaeIntakeRight.configure(algaeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        algaeAngulation1.setNeutralMode(NeutralMode.Brake);
        algaeAngulation2.setNeutralMode(NeutralMode.Brake);
        algaeAngulation2.setInverted(true);
        algaeAngulation2.follow(algaeAngulation1);

    }

    /**
     * @param position put in degrees
     */
    // public void setAlgaeAngularPosition(double position){
    //     double power = coralIntakePID.calculate(algaeAngulationEncoder.getPosition(), position);
    //     angulationAlgaeSetPower(power);
    // }

    public void angulationAlgaeSetPower(double power){
        algaeAngulation1.set(ControlMode.PercentOutput, power);
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
        //SmartDashboard.putNumber("Algae Angulation Value", algaeAngulationEncoder.getPosition());
        SmartDashboard.putNumber("Coral Angulation Value", coralAngulationEncoder.getPosition());
    }

}
