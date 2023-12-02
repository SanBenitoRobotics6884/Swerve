package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.Swerve.*;

public class SwerveModule {

  private CANSparkMax m_driveMotor;
  private CANSparkMax m_steerMotor;

  private CANCoder m_steerAbsoluteEncoder;
  private RelativeEncoder m_steerIntegratedEncoder;
  private RelativeEncoder m_driveEncoder;

  private SimpleMotorFeedforward m_driveFeedforward;

  private Rotation2d m_angleReference;
  private double m_velocityReference; // meters per second
    
  public SwerveModule(int driveID, int steerID, int encoderID, boolean driveInverted, 
                      boolean steerInverted, double magnetOffset) {
    m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    m_steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);

    m_driveMotor.setInverted(driveInverted);
    m_steerMotor.setInverted(steerInverted);

    m_steerAbsoluteEncoder = new CANCoder(encoderID);
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.magnetOffsetDegrees = magnetOffset;
    config.sensorCoefficient = 1.0 / 4096.0;
    config.sensorDirection = !steerInverted;
    config.unitString = "rot";
    m_steerAbsoluteEncoder.configAllSettings(config);

    m_steerIntegratedEncoder = m_steerMotor.getEncoder();
    m_steerIntegratedEncoder.setPositionConversionFactor(STEER_POSITION_CONVERSION);
    m_steerIntegratedEncoder.setPosition(m_steerAbsoluteEncoder.getAbsolutePosition());

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPosition(0);
    m_driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION);
    m_driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION);
    
    SparkMaxPIDController steerController = m_steerMotor.getPIDController();
    steerController.setPositionPIDWrappingMinInput(0);
    steerController.setPositionPIDWrappingMaxInput(1.0); // rotations
    steerController.setPositionPIDWrappingEnabled(true);
    steerController.setP(STEER_kP);
    steerController.setI(STEER_kI);
    steerController.setD(STEER_kD);
    m_steerMotor.setClosedLoopRampRate(STEER_RAMP_RATE);

    SparkMaxPIDController driveController = m_driveMotor.getPIDController();
    driveController.setP(DRIVE_kP);
    driveController.setI(DRIVE_kI);
    driveController.setD(DRIVE_kD);
    m_driveMotor.setClosedLoopRampRate(DRIVE_RAMP_RATE);

    m_driveFeedforward = new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV);

    m_angleReference = new Rotation2d();
  }

  public void setState(SwerveModuleState state) {
    m_angleReference = state.angle;
    m_velocityReference = state.speedMetersPerSecond;

    m_steerMotor.getPIDController().setReference(m_angleReference.getRotations(), ControlType.kPosition);
    m_driveMotor.getPIDController().setReference(
        m_velocityReference, ControlType.kVelocity, 0, 
        m_driveFeedforward.calculate(m_velocityReference));
  }

  public void setIntegratedEncoderPositionToAbsoluteEncoderMeasurement() {
    m_steerIntegratedEncoder.setPosition(m_steerAbsoluteEncoder.getAbsolutePosition());
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(m_steerAbsoluteEncoder.getAbsolutePosition());
  }

  public Rotation2d getDesiredRotation2d() {
    return m_angleReference;
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), getRotation2d());
  }

  public double getDriveEncoderPosition() {
    return m_driveEncoder.getPosition();
  }

  public double getAngleDegrees() {
    return Units.rotationsToDegrees(m_steerIntegratedEncoder.getPosition());
  }

  public double getDesiredAngleDegrees() {
    return m_angleReference.getDegrees();
  }

  public double getVelocity() {
    return m_driveEncoder.getVelocity();
  }

  public double getDesiredVelocity() {
    return m_velocityReference;
  }

  public void setCANCoderOffsetDegrees(double degrees) {
    m_steerAbsoluteEncoder.configMagnetOffset(degrees);
  }

  public double getCANCoderOffsetDegrees() {
    return m_steerAbsoluteEncoder.configGetMagnetOffset();
  }

  public void putData(String name) {
    SmartDashboard.putNumber(name + " rotation", m_steerAbsoluteEncoder.getAbsolutePosition());
    SmartDashboard.putNumber(name + " vel", getVelocity());
    SmartDashboard.putNumber(name + " desired vel", getDesiredVelocity());
  }

}
