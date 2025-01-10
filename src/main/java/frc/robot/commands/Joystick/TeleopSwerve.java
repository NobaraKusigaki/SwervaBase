package frc.robot.commands.Joystick;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Adicionado para compatibilidade
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

public class TeleopSwerve extends Command {

  private final SwerveSubsystem subSwerve;
  private final Supplier<Double> x, y, turning;
  private final SlewRateLimiter xLimiter, yLimiter, tLimiter;

  public TeleopSwerve(SwerveSubsystem subSwerve, Supplier<Double> x, Supplier<Double> y, Supplier<Double> turning, 
                      SlewRateLimiter xLimiter, SlewRateLimiter yLimiter, SlewRateLimiter tLimiter) {
    this.subSwerve = subSwerve;
    this.x = x;
    this.y = y;
    this.turning = turning;

    this.xLimiter = new SlewRateLimiter(Constants.kMaxAcelleration); 
    this.yLimiter = new SlewRateLimiter(Constants.kMaxAcelleration);
    this.tLimiter = new SlewRateLimiter(Constants.kMaxAngularAcelleration);

    addRequirements(subSwerve);
  }
  
  @Override
  public void initialize() {
    DataLogManager.start();  
  }

  @Override
  public void execute() {
    double xAxis = x.get();
    double yAxis = y.get();
    double turningAxis = turning.get();

    SmartDashboard.putNumber("Joystick X", xAxis);
    SmartDashboard.putNumber("Joystick Y", yAxis);
    SmartDashboard.putNumber("Joystick Turning", turningAxis);
    
    if (Math.abs(xAxis) > Constants.kDeadband || Math.abs(yAxis) > Constants.kDeadband || Math.abs(turningAxis) > Constants.kDeadband) {
      xAxis = xLimiter.calculate(xAxis) * Constants.kMetersPerSec;
      yAxis = yLimiter.calculate(yAxis) * Constants.kMetersPerSec;
      turningAxis = tLimiter.calculate(turningAxis) * Constants.kRadiansPerSec;

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(yAxis, xAxis, turningAxis);
      SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      subSwerve.setModuleStates(moduleStates);
  } else {
      subSwerve.stopModules(); 
  } 
  }

  @Override
  public void end(boolean interrupted) {
    subSwerve.stopModules(); 
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}