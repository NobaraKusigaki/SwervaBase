package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;

public class SwerveSubsystem extends SubsystemBase {
    private final Pigeon2 gyro;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final SwerveModule[] modules;
    private final SwerveModuleState[] states;
    private final StructArrayPublisher<SwerveModuleState> publisher;
    private final StringLogEntry moduleStatesLogEntry;

    private double[] angleRaw, angleDegress, angleRadian;
    private final DoubleArrayPublisher angleRawPbl, angleDegressPbl, angleRadianPbl;

    public SwerveSubsystem() {
        
        gyro = new Pigeon2(Constants.PIGEON_ID);
        resetGyro();

        DataLog log_subsytem = DataLogManager.getLog();
        moduleStatesLogEntry = new StringLogEntry(log_subsytem, "SwerveSubsystem/module states: ");

        modules = new SwerveModule[]{
            new SwerveModule(Constants.MOTOR_LEFT_DRIVER_FRONT, Constants.MOTOR_LEFT_ANGULAR_FRONT, Constants.CANCODER_FRONT_LEFT),
            new SwerveModule(Constants.MOTOR_RIGHT_DRIVER_FRONT, Constants.MOTOR_RIGHT_ANGULAR_FRONT, Constants.CANCODER_FRONT_RIGHT),
            new SwerveModule(Constants.MOTOR_LEFT_DRIVER_BACK, Constants.MOTOR_LEFT_ANGULAR_BACK, Constants.CANCODER_BACK_LEFT),
            new SwerveModule(Constants.MOTOR_RIGHT_DRIVER_BACK, Constants.MOTOR_RIGHT_ANGULAR_BACK, Constants.CANCODER_BACK_RIGHT)
        };

        kinematics = Constants.kDriveKinematics;
        states = new SwerveModuleState[modules.length];

        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
        
        angleRawPbl = NetworkTableInstance.getDefault().getDoubleArrayTopic("Angle Brute Values").publish();
        angleDegressPbl = NetworkTableInstance.getDefault().getDoubleArrayTopic("Angle Degress Values").publish();
        angleRadianPbl = NetworkTableInstance.getDefault().getDoubleArrayTopic("Angle Radian Values").publish();


        initializeModuleStates();

        angleRaw = new double[modules.length];
        angleDegress = new double[modules.length];
        angleRaw = new double[modules.length];

        odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), getModulePositions());
    }

    private void resetGyro() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.reset();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }).start();
    }

    private void initializeModuleStates() {
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        StringBuilder statesLog = new StringBuilder();
        for (SwerveModule module : modules) {
            statesLog.append(module.getState().toString()).append("\n");
        }

        for(int i = 0; i < modules.length; i++){
            angleRaw[i] = modules[i].CANcoderValue();
            angleDegress[i] = modules[i].CANcoderValueInDegrees();
            //angleRadian[i] = modules[i].CANcoderValueInRadians();
        }

        moduleStatesLogEntry.append(statesLog.toString());

        odometry.update(getRotation2d(), getModulePositions());
        updateSmartDashboard();

        publisher.set(states);
        angleRawPbl.set(angleRaw);
        angleDegressPbl.set(angleDegress);
        //angleRadianPbl.set(angleRadian);
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Robot Heading", getRotation2d().getDegrees());
        SmartDashboard.putNumber("Max Speed", getMaxSpeed());

        for (int i = 0; i < modules.length; i++) {
            SmartDashboard.putNumber("CANCoder Angle Module " + i, modules[i].CANcoderValueInDegrees());
        }
    }

    public double getMaxSpeed() {
        double maxSpeed = 0.0;

        for (SwerveModule module : modules) {
            double moduleSpeed = module.getState().speedMetersPerSecond;
            maxSpeed = Math.max(maxSpeed, Math.abs(moduleSpeed));
        }

        return Math.min(maxSpeed, 1.0);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getRotation2d())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleStates);
}
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    public double getRobotHeading() {
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getRobotHeading());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = new SwerveModulePosition(
                modules[i].getDriveMotorPosition(),
                Rotation2d.fromDegrees(modules[i].CANcoderValueInDegrees())
            );
        }
        return positions;
    }

    public void stopModules() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public SwerveModule[] getModule() {
        return modules;
    }
}