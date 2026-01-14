package frc.robot.subsystems.turret;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry txEntry = limelightTable.getEntry("tx");
    private final NetworkTableEntry tvEntry = limelightTable.getEntry("tv");

    public Turret() {
        SmartDashboard.putBoolean("Turret Auto Aim", false); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Turret Auto Aim", false);
        SmartDashboard.putBoolean("Turret Has Target", hasTarget());
        SmartDashboard.putNumber("Turret tx", getTx());
    }

    public boolean hasTarget() {
        return tvEntry.getDouble(0.0) >= 1.0;
    }

    public double getTx() {
        return txEntry.getDouble(0.0);
    }

    public double getEncoderDegrees() {
        // No encoder present. Return 0.0 or change callers to use robot yaw if needed.
        return 0.0;
    }
}