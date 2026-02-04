package frc.robot.subsystems.ArmStuff;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase implements ArmConstants  {

    public SparkMax coralShooter1;
    public SparkMax coralShooter2;
    private static final double MAX_RPM = 6000.0;

    public Shooter(){
        coralShooter1 = new SparkMax(shooterId1, MotorType.kBrushless);
        coralShooter2 = new SparkMax(shooterId2, MotorType.kBrushless);
    }

    public void setSpeed(double s){
        coralShooter1.set(s);
        coralShooter2.set(s * -1);
    }
    
    public void setRPM(double rpm) {
        if (Double.isNaN(rpm) || Double.isInfinite(rpm)) {
            setSpeed(0.0);
            return;
        }

        // Clamp target
        double boundedRPM = Math.max(-MAX_RPM, Math.min(MAX_RPM, rpm));
        double percent = boundedRPM / MAX_RPM;

        setSpeed(percent);
    }

    public double getIntakeCurrent() {
        return coralShooter1.getOutputCurrent();
    }


    
}
