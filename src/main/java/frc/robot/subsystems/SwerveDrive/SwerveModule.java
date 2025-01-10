package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

    private final SparkMax driveMotor;
    private final SparkMax angMotor;
    private final CANcoder cancoder;
    private final PIDController anglePID;

    private double lastSpeed = 0;
    private double lastPosition = 0;

    
    public SwerveModule(int driveMotorID, int angleMotorID, int cancoderID) {
        driveMotor = new SparkMax(driveMotorID, SparkMax.MotorType.kBrushless);
        angMotor = new SparkMax(angleMotorID, SparkMax.MotorType.kBrushless);
        cancoder = new CANcoder(cancoderID);
        anglePID = new PIDController(Constants.KP_Swerve, Constants.KI_Swerve, Constants.KD_Swerve);
    
        initMotors();
    }

    private void initMotors() {
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        
    }

    // Returns current state of the module (speed and angle)
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveSpd(), new Rotation2d(CANcoderValueInRadians()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state.optimize(getState().angle);

        driveMotor.set(state.speedMetersPerSecond / Constants.kMaxAcelleration);
        angMotor.set(anglePID.calculate(CANcoderValueInRadians(), state.angle.getRadians()));
    }

    public double CANcoderValueInRadians() {
        double rot = cancoder.getAbsolutePosition().getValueAsDouble();
        return Units.rotationsToRadians(rot);
    }

    public double getDriveSpd() {
        double currentSpeed = cancoder.getVelocity().getValueAsDouble();
        if (Math.abs(currentSpeed - lastSpeed) > 0.01) {
            lastSpeed = currentSpeed;
        }
        return lastSpeed;
    }
    public double getDriveMotorPosition() {
        double currentPosition = cancoder.getPosition().getValueAsDouble();
        if (Math.abs(currentPosition - lastPosition) > 0.01) {
            lastPosition = currentPosition;
        }
        return lastPosition;
    }

    public void stop() {
        driveMotor.set(0);
        angMotor.set(0);
    }

    public void resetPosition() {
  
    double reset = anglePID.calculate(CANcoderValueInRadians(), 0);
    angMotor.set(reset);
  }
}