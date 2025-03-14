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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;

public class IntakeSubsystem extends SubsystemBase {

    // Motors
    private SparkMax angulationCoral;
    private VictorSPX coralIntake;

    // PID Controller
    private PIDController coralIntakePID;

    // Encoders and Triggers
    private RelativeEncoder angulationEncoder;
    private DigitalInput limitTrigger;
    private DigitalInput coralDetector;

    // Configurators
    private SparkMaxConfig angulationCoralConfig;

    public IntakeSubsystem() {
        angulationCoral = new SparkMax(Constants.kIntakeAngularCoralMotorID, MotorType.kBrushless);
        angulationCoralConfig = new SparkMaxConfig();
        coralIntake = new VictorSPX(Constants.kIntakeCoralMotorID);

        coralIntakePID = new PIDController(Constants.kCoralIntakeKP, Constants.kCoralIntakeKI,
                Constants.kCoralIntakeKD);
        coralIntakePID.enableContinuousInput(0, 0.02);

        //algaeAngulationEncoder = angulationCoralMotor.getAlternateEncoder();
        //angulationCoralConfig.alternateEncoder.countsPerRevolution(360);
        
        angulationCoralConfig.idleMode(IdleMode.kBrake);
        angulationCoral.configure(angulationCoralConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        angulationEncoder = angulationCoral.getEncoder();
        limitTrigger = new DigitalInput(Constants.kMicroSwitchPWMPort);
        coralDetector = new DigitalInput(Constants.kMicroSwitchPWMPort-1);

    }

    /**
     * @param position put in degrees
     */
    public void setCoralAngularPosition(double position) {
        double power = coralIntakePID.calculate(angulationEncoder.getPosition(), position);
        angulationCoralSetPower(power);
    }

    public void angulationCoralSetPower(double power) {
        angulationCoral.set(power);
    }

    public void coralDisable() {
        coralIntake.set(ControlMode.PercentOutput, 0);
    }

    public void coralIntake(double power) {
        coralIntake.set(ControlMode.PercentOutput, power);
    }

    public void controlCoralAngulationWithAnalog(double yAxisValue) {
        double adjustedValue = -yAxisValue; 
        SmartDashboard.putNumber("coralAngulationPower", adjustedValue);
    
        if (Math.abs(adjustedValue) > Constants.kControllerDeadband) {
            if(!limitTrigger.get())
                angulationCoralSetPower(adjustedValue/10);
            else{
                angulationCoralSetPower(MathUtil.clamp(adjustedValue, -1, 0));
            }
        } else {
            angulationCoralSetPower(0);
        }
    }
    
        
    public void controlCoralAngulationWithPOV(int pov) {
        if (pov == 0) {
            angulationCoralSetPower(0.10);
        } else if ((pov == 180) && !limitTrigger.get()) {
            angulationCoralSetPower(-0.10);
        } else {
            angulationCoralSetPower(0);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("DetectorSwitchCoral", coralDetector.get());
        SmartDashboard.putBoolean("TriggerSwitchCoral", limitTrigger.get());
        SmartDashboard.putNumber("Coral Angulation Value", angulationEncoder.getPosition());
    }
}