package frc.robot.commands;

import java.util.Collections;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.TurretConstants;

public class AimTurretCommand extends Command {
    private final Swerve swerve;
    private final PIDController pid;
    private final double maxOutput;
    private final Set<Subsystem> requirements;
    private final Supplier<Translation2d> translationSupplier;

    private final NetworkTableEntry txEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    private final NetworkTableEntry tvEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");

    public AimTurretCommand(Swerve swerve, Supplier<Translation2d> translationSupplier) {
        this.swerve = swerve;
        this.pid = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        this.maxOutput = TurretConstants.MAX_OUTPUT;
        this.pid.setTolerance(1.0);
        this.requirements = Collections.singleton((Subsystem) this.swerve);
        this.translationSupplier = translationSupplier;
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {
        boolean hasTarget = tvEntry.getDouble(0.0) >= 1.0;
        if (!hasTarget) {
            swerve.drive(translationSupplier.get(), 0.0, false, true);
            return;
        }

        double tx = txEntry.getDouble(0.0);
        double errorDeg = tx - TurretConstants.CAMERA_TURRET_ANGLE_OFFSET_DEG;

        double rotCmd = pid.calculate(errorDeg, 0.0);
        rotCmd = MathUtil.clamp(rotCmd, -maxOutput, maxOutput);

        swerve.drive(translationSupplier.get(), rotCmd, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0.0, 0.0), 0.0, false, true);
        pid.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}