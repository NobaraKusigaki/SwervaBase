package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

    private final SparkMax driveMotor;
    private final SparkMax angMotor;
    private final CANcoder cancoder;
    private final PIDController anglePID;

    private double lastSpeed = 0;
    private double lastPosition = 0;
    private Rotation2d lastAngle = new Rotation2d(0); 

    public SwerveModule(int driveMotorID, int angleMotorID, int cancoderID) {
        driveMotor = new SparkMax(driveMotorID, SparkMax.MotorType.kBrushless);
        angMotor = new SparkMax(angleMotorID, SparkMax.MotorType.kBrushless);
        cancoder = new CANcoder(cancoderID);
        anglePID = new PIDController(Constants.KP_Swerve_ANGLE, Constants.KI_Swerve_ANGLE, Constants.KD_Swerve_ANGLE);
    
        initMotors();
    }

    private void initMotors() {
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveSpd(), new Rotation2d(CANcoderValueInDegrees()));
    }

    private SwerveModuleState optimizeState(SwerveModuleState desiredState) {
        double currentAngle = lastAngle.getDegrees();
        double targetAngle = desiredState.angle.getDegrees();

        double delta = targetAngle - currentAngle;
        if (Math.abs(delta) > 90) {
            delta = (delta + 180) % 360 - 180; 
            desiredState = new SwerveModuleState(-desiredState.speedMetersPerSecond, Rotation2d.fromDegrees(currentAngle + delta));
        }

        return desiredState;
    }
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = optimizeState(state);

        driveMotor.set(state.speedMetersPerSecond / Constants.kMaxAcelleration);
        angMotor.set(anglePID.calculate(CANcoderValueInDegrees(), state.angle.getRadians()));

        lastAngle = state.angle;
    }

    public double CANcoderValueInDegrees() {
        double rot = cancoder.getAbsolutePosition().getValueAsDouble();
        return Units.rotationsToDegrees(rot);
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

    public void setRotationSpeed(double speed) {
        angMotor.set(speed); 
    }
}