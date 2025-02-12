package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged (name = "IntakeSubsystem")
public class IntakeSubsystem extends SubsystemBase{

    //Motors
    private SparkMax angulationMotor;
    private SparkMax algaeIntakeLeft;
    private SparkMax algaeIntakeRight;
    private SparkMax coralIntake;

    //PID Controller
    private PIDController intakePID;

    //Encoders
    private RelativeEncoder angulationEncoder;

    //Configurators
    private SparkMaxConfig angulationConfig;
    private SparkMaxConfig algaeLeftConfig;
    private SparkMaxConfig algaeRightConfig;
    private SparkMaxConfig coralConfig;

    public IntakeSubsystem(){
        angulationMotor = new SparkMax(Constants.intakeAngularMotor, MotorType.kBrushed);
        algaeIntakeLeft = new SparkMax(Constants.intakeAlgaeLeft, MotorType.kBrushless);
        algaeIntakeRight = new SparkMax(Constants.intakeAlgaeRight, MotorType.kBrushless);
        coralIntake = new SparkMax(Constants.intakeCoral, MotorType.kBrushless);

        intakePID = new PIDController(Constants.intakeKP, Constants.intakeKI, Constants.intakeKD);
        intakePID.enableContinuousInput(0, 0.02);

        //angulationEncoder = angulationMotor.getAlternateEncoder();
        angulationConfig.alternateEncoder.countsPerRevolution(360);
        angulationConfig.idleMode(IdleMode.kBrake);
        angulationMotor.configure(angulationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        algaeLeftConfig.inverted(false);
        algaeLeftConfig.idleMode(IdleMode.kBrake);
        algaeRightConfig.inverted(true);
        algaeRightConfig.idleMode(IdleMode.kBrake);

        algaeIntakeLeft.configure(algaeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algaeIntakeRight.configure(algaeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        coralConfig.idleMode(IdleMode.kBrake);

        coralIntake.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * @param position put in degrees
     */
    public void setAngularPosition(double position){
        double power = intakePID.calculate(angulationEncoder.getPosition(), position);
        angulationSetPower(power);
    }

    private void angulationSetPower(double power){
        angulationMotor.set(power);
    }

    public void algaeIntakeDisable(){
        algaeIntakeLeft.set(0);
        algaeIntakeRight.set(0);
    }

    public void algaeIntake(double power, boolean input){
        if(input){
            algaeIntakeLeft.set(power);
            algaeIntakeRight.set(-power);
        }else{
            algaeIntakeLeft.set(-power);
            algaeIntakeRight.set(power);
        }
    }

    public void coralIntake(double power){
        coralIntake.set(power);
    }

    public void coralDisable(){
        coralIntake.set(0);
    }

}
