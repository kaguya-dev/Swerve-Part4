package frc.robot.Subsystems.Sensors;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;

public class IMUSubsystem extends SubsystemBase{
    private Pigeon2 imu;
    private Angle yaw,pitch,roll;

    public IMUSubsystem(){
        imu = new Pigeon2(Constants.kPigeonID);
        updateIMUValues();
    }

    private void updateIMUValues(){
        yaw = imu.getYaw().getValue();
        pitch = imu.getPitch().getValue();
        roll = imu.getRoll().getValue();
    }

    public void resetYaw(){
        imu.setYaw(0);
    }

    @Override
    public void periodic(){
        updateIMUValues();;
    }
    
    public Angle getYaw() {
        return yaw;
    }

    public Angle getPitch() {
        return pitch;
    }

    public Angle getRoll() {
        return roll;
    }

    public boolean getIMUAvaliable(){
        return imu.isConnected();
    }

}
