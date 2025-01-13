package frc.robot.commands.Reset;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class WheelsReset extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController anglePID;
    private boolean resetInProgress;

    public WheelsReset(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.anglePID = new PIDController(Constants.KP_Swerve_ANGLE, Constants.KI_Swerve_ANGLE, Constants.KD_Swerve_ANGLE);
        addRequirements(swerveSubsystem);
        this.resetInProgress = true;
    }

    @Override
    public void initialize() {
        swerveSubsystem.stopModules();
    }

    @Override
    public void execute() {
        boolean allAtTarget = true;

        for (int i = 0; i < swerveSubsystem.getModule().length; i++) {
            double currentAngle = swerveSubsystem.getModule()[i].CANcoderValueInDegrees();
            double targetAngle = 0; // Reset target angle to 0

            if (Math.abs(currentAngle - targetAngle) <= 1) {
                // Stop individual module
                swerveSubsystem.getModule()[i].stop();
            } else {
                allAtTarget = false;
                double rotationSpeed = MathUtil.clamp(anglePID.calculate(currentAngle, targetAngle), 0.05, 0.05);
                swerveSubsystem.getModule()[i].setRotationSpeed(rotationSpeed);
            }

            // Debugging: Log angles to SmartDashboard
            SmartDashboard.putNumber("Module " + i + " Angle", currentAngle);
        }

        if (allAtTarget && resetInProgress) {
            resetInProgress = false;
        }
    }

    @Override
    public boolean isFinished() {
        return !resetInProgress;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
}