package frc.robot.commands.Reset;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

public class WheelsReset extends Command {

    // Declare subsystem
    private final SwerveSubsystem subswerve;

    // Constructor
    public WheelsReset(SwerveSubsystem subswerve) {
        this.subswerve = subswerve;
        addRequirements(subswerve); 
    }

    // Command initialization
    @Override
    public void initialize() {
        
        subswerve.resetModules();
    }

    
    @Override
    public void execute() {}

    
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true; // Ensures the command completes after initialization
    }
}