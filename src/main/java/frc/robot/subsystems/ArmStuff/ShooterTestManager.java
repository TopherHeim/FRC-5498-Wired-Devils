package frc.robot.subsystems.ArmStuff;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.Instant;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmStuff.Shooter;
import frc.robot.subsystems.swerve.DistanceInterpolator;


public class ShooterTestManager {
    private final Shooter shooter;
    private final DistanceInterpolator interpolator;

    // Limelight table and entries
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tyEntry;

    // SmartDashboard entries (we'll mirror as NetworkTable entries)
    private final String kUseMapKey = "ShooterTest/UseMap";
    private final String kManualRPMKey = "ShooterTest/ManualRPM";
    private final String kCmdRPMKey = "ShooterTest/CommandedRPM";
    private final String kDistanceKey = "ShooterTest/DistanceMeters";
    private final String kLogButtonKey = "ShooterTest/LogShot";

    // Camera mounting constants (fill with your real values)
    private final double cameraHeightMeters;     // height of limelight lens from floor (meters)
    private final double targetHeightMeters;     // height of target center from floor (meters)
    private final double cameraAngleDegrees;     // mounting angle above horizontal (degrees)

    // Log file on RoboRIO (deploy directory)
    private final File logFile;

    public ShooterTestManager(Shooter shooter,
                              double cameraHeightMeters,
                              double targetHeightMeters,
                              double cameraAngleDegrees) {
        this.shooter = shooter;
        this.interpolator = new DistanceInterpolator();

        this.cameraHeightMeters = cameraHeightMeters;
        this.targetHeightMeters = targetHeightMeters;
        this.cameraAngleDegrees = cameraAngleDegrees;

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        this.limelightTable = nt.getTable("limelight");
        this.tyEntry = limelightTable.getEntry("ty"); // vertical offset degrees

        // Initialize SmartDashboard values
        SmartDashboard.putBoolean(kUseMapKey, true);
        SmartDashboard.putNumber(kManualRPMKey, 2000.0);
        SmartDashboard.putNumber(kCmdRPMKey, 0.0);
        SmartDashboard.putNumber(kDistanceKey, 0.0);
        SmartDashboard.putBoolean(kLogButtonKey, false);

        // create/open log file in deploy directory
        File deployDir = Filesystem.getDeployDirectory();
        logFile = new File(deployDir, "shooter_test_log.csv");
        ensureLogHasHeader();
    }

    private void ensureLogHasHeader() {
        if (!logFile.exists()) {
            try (FileWriter w = new FileWriter(logFile, true)) {
                w.append("timestamp,distance_m,commanded_rpm,use_map,notes\n");
                w.flush();
            } catch (IOException e) {
                System.err.println("Failed to create shooter log file: " + e.getMessage());
            }
        }
    }

    
    public void periodic() {
        // Read limelight ty (degrees)
        double ty = tyEntry.getDouble(0.0);

        // Compute distance using the standard formula:
        // distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
        double angleToTargetDeg = cameraAngleDegrees + ty;
        double distanceMeters;
        if (Math.abs(Math.tan(Math.toRadians(angleToTargetDeg))) < 1e-6) {
            distanceMeters = 0.0;
        } else {
            distanceMeters = (targetHeightMeters - cameraHeightMeters)
                    / Math.tan(Math.toRadians(angleToTargetDeg));
        }

        // Publish measured distance
        SmartDashboard.putNumber(kDistanceKey, distanceMeters);

        // Choose commanded RPM (map or manual)
        boolean useMap = SmartDashboard.getBoolean(kUseMapKey, true);
        double commandedRPM;
        if (useMap) {
            commandedRPM = interpolator.getRPMForDistance(distanceMeters);
        } else {
            commandedRPM = SmartDashboard.getNumber(kManualRPMKey, 2000.0);
        }

        // Publish commanded RPM
        SmartDashboard.putNumber(kCmdRPMKey, commandedRPM);

        // Command the shooter (uses Shooter.setRPM helper)
        shooter.setRPM(commandedRPM);

        // Check for log button press (SmartDashboard boolean)
        boolean logPressed = SmartDashboard.getBoolean(kLogButtonKey, false);
        if (logPressed) {
            // append a CSV row
            appendLogRow(distanceMeters, commandedRPM, useMap, "");
            // reset the dashboard button to false so it behaves like a momentary button
            SmartDashboard.putBoolean(kLogButtonKey, false);
        }
    }

    private void appendLogRow(double distanceMeters, double commandedRPM, boolean useMap, String notes) {
        String ts = Instant.now().toString();
        String line = String.format("%s,%.4f,%.1f,%b,%s\n", ts, distanceMeters, commandedRPM, useMap, notes.replace(',', ';'));
        try (FileWriter w = new FileWriter(logFile, true)) {
            w.append(line);
            w.flush();
            System.out.println("Logged shooter test row: " + line.trim());
        } catch (IOException e) {
            System.err.println("Failed to append shooter log: " + e.getMessage());
        }
    }

    
    public DistanceInterpolator getInterpolator() {
        return interpolator;
    }

    
    public String getLogFilePath() {
        return logFile.getAbsolutePath();
    }
}
