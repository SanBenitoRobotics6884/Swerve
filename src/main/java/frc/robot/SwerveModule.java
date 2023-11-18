package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.Swerve.*;

public class SwerveModule {
  private CANSparkMax m_driveMotor;
  private CANSparkMax m_steerMotor;

  private CANCoder m_steerEncoder;
  private RelativeEncoder m_driveEncoder;

  private PIDController m_steerController;

  private Rotation2d m_angleMeasurement;
  private Rotation2d m_angleReference;

  private double m_speedPercentOutput;
    
  public SwerveModule(int driveID, int steerID, int encoderID, boolean driveInverted, 
                      boolean steerInverted, double magnetOffset) {
    m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    m_steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);

    m_driveMotor.setInverted(driveInverted);
    m_steerMotor.setInverted(steerInverted);

    m_steerEncoder = new CANCoder(encoderID);
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.magnetOffsetDegrees = magnetOffset;
    config.sensorCoefficient = 1.0 / 4096.0;
    config.sensorDirection = !steerInverted;
    config.unitString = "rot";
    m_steerEncoder.configAllSettings(config);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPosition(0);
    m_driveEncoder.setPositionConversionFactor(POSITION_CONVERSION);
    m_driveEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION);

    m_steerController = new PIDController(STEER_kP, STEER_kI, STEER_kD);
    m_steerController.enableContinuousInput(0, 1.0);

    m_angleMeasurement = new Rotation2d();
    m_angleReference = new Rotation2d();
  }

  public void setState(SwerveModuleState state) {
    m_angleMeasurement = Rotation2d.fromRotations(m_steerEncoder.getAbsolutePosition());
    m_angleReference = state.angle;

    m_steerMotor.set(m_steerController.calculate(
        m_angleMeasurement.getRotations(),
        m_angleReference.getRotations()));

    m_driveMotor.set(MAX_OUTPUT * state.speedMetersPerSecond);

    m_speedPercentOutput = state.speedMetersPerSecond;
  }

  public Rotation2d getAngleMeasurement() {
    return m_angleMeasurement;
  }

  public Rotation2d getAngleReference() {
    return m_angleReference;
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), getAngleMeasurement());
  }

  public double getAngleDegrees() {
    return m_angleMeasurement.getDegrees();
  }

  public double getDesiredAngleDegrees() {
    return m_angleReference.getDegrees();
  }

  public double getVelocity() {
    return m_driveEncoder.getVelocity();
  }

  public double getSpeedPercentOutput() {
    return m_speedPercentOutput;
  }

}
