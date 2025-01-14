package frc.robot.commands.Reset;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class WheelsReset extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController anglePID;
    private double[] pidspeed = {0,0,0,0};
    private double[] pidConverted = {0,0,0,0};
    private final DoubleArrayPublisher pidPublisher;
    private final DoubleArrayPublisher convertedPublisher;
    private boolean resetInProgress;

    public WheelsReset(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.anglePID = new PIDController(Constants.KP_Swerve_ANGLE, Constants.KI_Swerve_ANGLE, Constants.KD_Swerve_ANGLE);
        addRequirements(swerveSubsystem);
        pidPublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("ResetSpeeds").publish();
        convertedPublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("ResetConvertedSpeeds").publish();
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
            pidspeed[i] = anglePID.calculate(currentAngle, targetAngle);

            if (Math.abs(currentAngle - targetAngle) <= 1) {
                // Stop individual module
                swerveSubsystem.getModule()[i].stop();
            } else {
                allAtTarget = false;
                pidConverted[i] = Math.min(pidspeed[i], 0.2);
                swerveSubsystem.getModule()[i].setRotationSpeed(pidConverted[i]);
            }

            // Debugging: Log angles to SmartDashboard
            SmartDashboard.putNumber("Module " + i + " Angle", currentAngle);
        }

        if (allAtTarget && resetInProgress) {
            resetInProgress = false;
        }

        pidPublisher.set(pidspeed);
        convertedPublisher.set(pidConverted);
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