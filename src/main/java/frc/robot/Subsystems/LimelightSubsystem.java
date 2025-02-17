package frc.robot.Subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase{
    NetworkTable lime = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = lime.getEntry("tx");
    NetworkTableEntry ty = lime.getEntry("ty");
    NetworkTableEntry ta = lime.getEntry("ta");

    private double x = tx.getDouble(0.0);
    private double y = ty.getDouble(0.0);
    private double a = ta.getDouble(0.0);

    public LimelightSubsystem(){}

   // @Override
    public void periodic(){
       x  = tx.getDouble(0.0);
       y =  ty.getDouble(0.0);
       a = ta.getDouble(0.0);
    }

    public void setTargetID(int ID){
        LimelightHelpers.setPriorityTagID("limelight", ID);
    }

    public boolean getTargetLime(){
        if(a > 0) return true;
        return false;
    }

    public double getLimeXValue(){
        return x;
    }

    public double getLimeYValue(){
        return y;
    }

    public double getLimeAreaValue(){
        return a;
    }
        
}