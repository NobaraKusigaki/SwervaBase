package frc.robot.commands.Joystick;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.Reset.WheelsReset;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

public class TeleopSwerve extends Command {
  private final SwerveSubsystem subSwerve;
  private final Supplier<Double> x, y, turning;
  private final Supplier<Integer> povSupplier;
  private final Supplier<Boolean> robotCentral, buttonBPressed;
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter tLimiter = new SlewRateLimiter(3.0);

  private boolean resetScheduled = false;

  public TeleopSwerve(SwerveSubsystem subSwerve, Supplier<Double> x, Supplier<Double> y, 
                      Supplier<Double> turning, Supplier<Boolean> robotCentral, Supplier<Boolean> buttonBPressed,
                      Supplier<Integer> povSupplier) {
    this.subSwerve = subSwerve;
    this.x = x;
    this.y = y;
    this.povSupplier = povSupplier;
    this.turning = turning;
    this.robotCentral = robotCentral;
    this.buttonBPressed = buttonBPressed;

    addRequirements(subSwerve);
  }

  @Override
  public void initialize() {
    DataLogManager.start();
    resetScheduled = false;
  }

  @Override
  public void execute() {
    if (buttonBPressed.get()) {
      if (!resetScheduled) {
        CommandScheduler.getInstance().schedule(new WheelsReset(subSwerve));
        resetScheduled = true;
      }
    } else {
      resetScheduled = false; // Permitir que o reset seja reprogramado quando o botão for solto
      double xAxis = xLimiter.calculate(MathUtil.applyDeadband(x.get(), Constants.kDeadband)) * Constants.kMetersPerSec;
      double yAxis = yLimiter.calculate(MathUtil.applyDeadband(y.get(), Constants.kDeadband)) * Constants.kMetersPerSec;
      double turningAxis = tLimiter.calculate(MathUtil.applyDeadband(turning.get(), Constants.kDeadband)) * Constants.kRadiansPerSec;
      double pov = povSupplier.get();
      SmartDashboard.putNumber("Joystick X", xAxis);
      SmartDashboard.putNumber("Joystick Y", yAxis);
      SmartDashboard.putNumber("Joystick Turning", turningAxis);
      SmartDashboard.putNumber("POV", pov);

      if (Math.abs(xAxis) > 0 || Math.abs(yAxis) > 0 || Math.abs(turningAxis) > 0) {
        Translation2d translation = new Translation2d(xAxis, yAxis);

        ChassisSpeeds chassisSpeeds = robotCentral.get()
            ? new ChassisSpeeds(translation.getX(), translation.getY(), turningAxis)
            : ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), turningAxis, subSwerve.getRotation2d());

        SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        subSwerve.setModuleStates(moduleStates);
      } else {
        subSwerve.stopModules();
      }
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