// Copyright (c) FIRST and other WPILib contributors. 
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

  // Operator Constants
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // Drivetrain Geometry Constants
  public static final double kTrackWidth = 0.58;
  public static final double kWheelBase = 0.60;  
  public static final double kWheelDiameterMeters = 0.1016; // Diâmetro da roda em metros (4 polegadas convertidas para metros)
  public static final double kDriveMotorGearRatio = 6.75; // Relação de transmissão do motor de direção
  public static final double kTurningMotorGearRatio = 21.43; // Relação de transmissão do motor de rotação

  // Drivetrain Kinematics
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // Frente esquerda
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // Traseira esquerda
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Frente direita
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)  // Traseira direita
  );

  // Constants for Motion
  public static final double kDeadband = 0.05;
  public static final double kMaxAcelleration = 1.0;
  public static final double kMaxAngularAcelleration = Math.PI;
  public static final double kMetersPerSec = 4.5;
  public static final double kRadiansPerSec = Math.PI;
 public static final double  SLOW_ROTATION_SPEED = 0.05;
  // Motor & Encoder IDs
  public static final int MOTOR_LEFT_DRIVER_FRONT = 1;
  public static final int MOTOR_LEFT_DRIVER_BACK = 3;
  public static final int MOTOR_RIGHT_DRIVER_FRONT = 7;
  public static final int MOTOR_RIGHT_DRIVER_BACK = 5;
  public static final int MOTOR_LEFT_ANGULAR_FRONT = 2;
  public static final int MOTOR_LEFT_ANGULAR_BACK = 4;
  public static final int MOTOR_RIGHT_ANGULAR_FRONT = 8;
  public static final int MOTOR_RIGHT_ANGULAR_BACK = 6;

  // Joystick & Pigeon Configuration
  public static final int JOY_PORT = 0;
  public static final int PIGEON_ID = 15;

  // CANCoder IDs
  public static final int CANCODER_FRONT_LEFT = 11;
  public static final int CANCODER_FRONT_RIGHT = 12;
  public static final int CANCODER_BACK_LEFT = 13;
  public static final int CANCODER_BACK_RIGHT = 14;

  // PID Constants for Swerve
  public static final double KP_Swerve_ANGLE = 0.006;
  public static final double KI_Swerve_ANGLE = 0.0001;
  public static final double KD_Swerve_ANGLE = 0.001;

  // Joystick Button Mapping
  public static final int LEFT_STICK_Y = 1;
  public static final int LEFT_STICK_X = 0;
  public static final int RIGHT_ROT_AXIS = 4;

  public static final int BNT_B = 2;
  public static final int BNT_A = 1;
  public static final int BNT_X = 3;

  // Maximum Speed
  public static final double MAX_SPEED = 1;

  // Conversion Constants
  public static final double kDriveEncoderRot2Meter = (Math.PI * kWheelDiameterMeters) / kDriveMotorGearRatio;
  public static final double kTurningEncoderRot2Rad = (2 * Math.PI) / kTurningMotorGearRatio;
  public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
  public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

}