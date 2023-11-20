// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

import static frc.robot.Constants.Swerve.*;

import com.ctre.phoenix.sensors.Pigeon2;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveModule[] m_modules = new SwerveModule[] {
    new SwerveModule(
        FR_DRIVE_ID, FR_STEER_ID, FR_ENCODER_ID, 
        FR_DRIVE_INVERTED, FR_STEER_INVERTED, FR_OFFSET_DEGREES),
    new SwerveModule(
        FL_DRIVE_ID, FL_STEER_ID, FL_ENCODER_ID, 
        FL_DRIVE_INVERTED, FL_STEER_INVERTED, FL_OFFSET_DEGREES),
    new SwerveModule(
        BR_DRIVE_ID, BR_STEER_ID, BR_ENCODER_ID, 
        BR_DRIVE_INVERTED, BR_STEER_INVERTED, BR_OFFSET_DEGREES),
    new SwerveModule(
        BL_DRIVE_ID, BL_STEER_ID, BL_ENCODER_ID, 
        BL_DRIVE_INVERTED, BL_STEER_INVERTED, BL_OFFSET_DEGREES)};

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      FR_LOCATION,
      FL_LOCATION,
      BR_LOCATION,
      BL_LOCATION);

  private Pigeon2 m_gyro = new Pigeon2(PIGEON_ID);
  private Pose2d m_pose = new Pose2d();
  private SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4];

  private SwerveDriveOdometry m_odometry;

  // This is for advantage scope
  private DoubleArrayPublisher m_moduleMeasurementsPublisher = 
      NetworkTableInstance.getDefault().getTable("Module States").getDoubleArrayTopic("measurements").publish();
  private DoubleArrayPublisher m_moduleSetpointsPublisher = 
      NetworkTableInstance.getDefault().getTable("Module States").getDoubleArrayTopic("setpoints").publish();
  private DoublePublisher m_gyroAnglePublisher = 
      NetworkTableInstance.getDefault().getTable("Module States").getDoubleTopic("gyro angle").publish();
  private DoubleArrayPublisher m_posePublisher =
      NetworkTableInstance.getDefault().getTable("Odometry").getDoubleArrayTopic("pose").publish();
  private double[] m_moduleMeasurements = new double[8];
  private double[] m_moduleSetpoints = new double[8];
  private double m_gyroAngle;
  private double[] m_poseAsArray = new double[3];

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    for (int i = 0; i < 4; i++) {
      m_modulePositions[i] = m_modules[i].getModulePosition();
    }

    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        getAngle(),
        m_modulePositions,
        m_pose);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      m_modulePositions[i] = m_modules[i].getModulePosition();
    }
          // this is for advantage scope
    m_moduleMeasurements[0] = m_modules[1].getAngleDegrees(); // FL
    m_moduleMeasurements[1] = m_modules[1].getVelocity();     // FL
    m_moduleMeasurements[2] = m_modules[0].getAngleDegrees(); // FR
    m_moduleMeasurements[3] = m_modules[0].getVelocity();     // FR
    m_moduleMeasurements[4] = m_modules[3].getAngleDegrees(); // BL
    m_moduleMeasurements[5] = m_modules[3].getVelocity();     // BL
    m_moduleMeasurements[6] = m_modules[2].getAngleDegrees(); // BR
    m_moduleMeasurements[7] = m_modules[2].getVelocity();     // BR

    m_moduleSetpoints[0] = m_modules[1].getDesiredAngleDegrees(); // FL
    m_moduleSetpoints[1] = m_modules[1].getVelocity();            // FL
    m_moduleSetpoints[2] = m_modules[0].getDesiredAngleDegrees(); // FR
    m_moduleSetpoints[3] = m_modules[0].getVelocity();            // FR
    m_moduleSetpoints[4] = m_modules[3].getDesiredAngleDegrees(); // BL
    m_moduleSetpoints[5] = m_modules[3].getVelocity();            // BL
    m_moduleSetpoints[6] = m_modules[2].getDesiredAngleDegrees(); // BR
    m_moduleSetpoints[7] = m_modules[2].getVelocity();            // BR

    m_pose = m_odometry.update(getAngle(), m_modulePositions);

    m_poseAsArray[0] = m_pose.getX();
    m_poseAsArray[1] = m_pose.getY();
    m_poseAsArray[2] = m_pose.getRotation().getDegrees();

    // this is for advantage scope
    m_gyroAngle = m_gyro.getYaw();
    m_moduleMeasurementsPublisher.accept(m_moduleMeasurements);
    m_moduleSetpointsPublisher.accept(m_moduleSetpoints);
    m_gyroAnglePublisher.accept(m_gyroAngle);
    m_posePublisher.accept(m_poseAsArray);

    SmartDashboard.putNumber("FR", m_modules[0].getDegrees());
    SmartDashboard.putNumber("FL", m_modules[1].getDegrees());
    SmartDashboard.putNumber("BR", m_modules[2].getDegrees());
    SmartDashboard.putNumber("BL", m_modules[3].getDegrees());
  }

  public void driveRobotOriented(ChassisSpeeds speeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);

    for(int i = 0; i < 4; i++) {
      m_modules[i].setState(SwerveModuleState.optimize(
          states[i], m_modules[i].getAngleMeasurement()));
    }
  }

  public void driveFieldOriented(ChassisSpeeds speeds) {
    driveRobotOriented(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle()));
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public void zeroYaw() {
    m_gyro.setYaw(0);
  }

  public void setOffsetsToZero() {
    for (SwerveModule module : m_modules) {
      module.setCANCoderOffsetDegrees(0);
    }
  }

  public void setOffsetsToCANCoderMeasurement() {
    for (SwerveModule module : m_modules) {
      module.setCANCoderOffsetDegrees(module.getDegrees());
    }
  }

  public void printOffsets() {
    for (SwerveModule module : m_modules) {
      System.out.println(module.getCANCoderOffsetDegrees());
    }
  }

}
