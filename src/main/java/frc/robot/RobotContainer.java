package frc.robot;
import frc.robot.commands.Joystick.TeleopSwerve;
import frc.robot.commands.Reset.WheelsReset;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick joy = new Joystick(Constants.JOY_PORT);

  public RobotContainer() {
   swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem,
   ()-> -joy.getRawAxis(Constants.LEFT_STICK_Y),
   ()-> joy.getRawAxis(Constants.LEFT_STICK_X),
   ()-> joy.getRawAxis(Constants.RIGHT_ROT_AXIS), 
   ()-> joy.getRawButton(Constants.BNT_A),
   ()-> joy.getRawButton(Constants.BNT_B),
   () -> joy.getPOV()));

    configureBindings();
  }
  
  private void configureBindings() {
    //new JoystickButton(joy, 4)
    //  .onTrue(new WheelsReset(swerveSubsystem).withTimeout(0.5));
  }

  /*public Command getAutonomousCommand() {
   
  }*/
}