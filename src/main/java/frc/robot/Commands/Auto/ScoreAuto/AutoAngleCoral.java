package frc.robot.Commands.Auto.ScoreAuto;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class AutoAngleCoral {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput infrared;;
    private final double targetRotations;

    public AutoAngleCoral(int motorID, int infrared) {
        this.motor = new SparkMax(motorID, MotorType.kBrushless);
        this.encoder = motor.getEncoder();
        this.infrared = new DigitalInput(infrared);
        this.targetRotations = 8;
    }

    public void initialize() {
        encoder.setPosition(0); 
    }

    public void angleCoral() {
        if (encoder.getPosition() < targetRotations) {
            motor.set(0.4); //Verificar se é positivo ou negativo 
        } else {
            motor.set(0); 
        }
    }

    public boolean isAngulated() {
        return encoder.getPosition() >= targetRotations;
    }

    public boolean pieceOn() {
        return infrared.get();
    }
}