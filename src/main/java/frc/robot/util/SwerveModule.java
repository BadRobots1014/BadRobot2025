package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  // All the things
  private final SparkMax driveMotor;
  private final SparkMax turningMotor;

  public PIDController turningPidController;

  private final CANcoder absoluteEncoder;
  private final double absoluteEncoderOffsetRad;
  private final double absoluteEncoderOffsetRot;

  private ShuffleboardTab m_tab;
  private SwerveModuleState m_lastState = new SwerveModuleState();
  private SwerveModuleState m_lastStateOptimized = new SwerveModuleState();
  private double m_lastPIDOutput = 0;

  private GenericEntry p;
  private GenericEntry i;
  private GenericEntry d;

  public SwerveModule(
    int driveMotorId,
    int turningMotorId,
    boolean driveMotorReversed,
    boolean turningMotorReversed,
    int absoluteEncoderId,
    double absoluteEncoderOffset,
    boolean absoluteEncoderReversed,
    GenericEntry p,
    GenericEntry i,
    GenericEntry d
  ) {
    // Absolute encoder setup
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderOffsetRot =
      this.absoluteEncoderOffsetRad / (2 * Math.PI);
    absoluteEncoder = new CANcoder(absoluteEncoderId);
    CANcoderConfigurator configer = absoluteEncoder.getConfigurator();
    MagnetSensorConfigs config = new MagnetSensorConfigs();
    config.AbsoluteSensorDiscontinuityPoint = .5;
    config.MagnetOffset = absoluteEncoderOffsetRot;
    config.SensorDirection =
      absoluteEncoderReversed
        ? SensorDirectionValue.Clockwise_Positive
        : SensorDirectionValue.CounterClockwise_Positive;
    configer.apply(config);

    // Motor setup
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();

    driveConfig.inverted(absoluteEncoderReversed);
    turningConfig.inverted(turningMotorReversed);

    driveConfig.idleMode(IdleMode.kBrake);
    turningConfig.idleMode(IdleMode.kBrake);

    //relative encoder setup
    driveConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    turningConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    //set configuration
    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
    
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Setup PID controllers
    turningPidController =
      new PIDController(
        p.getDouble(1),
        i.getDouble(0),
        d.getDouble(0),
        ModuleConstants.kTurningPeriod
      );
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    turningPidController.reset();

    // Setup Shuffleboard
    m_tab = Shuffleboard.getTab("Swerve Modules");
    m_tab.addDouble("Last angle " + driveMotorId, this::getLastStateAngle);
    m_tab.addDouble("Last speed" + driveMotorId, this::getLastStateSpeed);
    m_tab.addDouble("Last angle optimized" + driveMotorId, this::getLastStateAngleOptimized);
    m_tab.addDouble("Last speed optimized" + driveMotorId, this::getLastStateSpeedOptimized);
    m_tab.addDouble("Encoder angle" + driveMotorId, this::getAbsoluteEncoderRad);
    m_tab.addDouble("Last PID Output" + driveMotorId, this::getLastPIDOutput);
    m_tab.addDouble("Last error" + driveMotorId, this::getLastError);

    // Reset the encoders on start
    resetEncoders();
  }

  public void resetEncoders() {
    driveMotor.getEncoder().setPosition(0);
    turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad()); // Reset to absolute position
  }

  public void setDesiredState(SwerveModuleState state) {

    if (
      Math.abs(state.speedMetersPerSecond) < ModuleConstants.kModuleDeadband
    ) {
      stop();
      return;
    }
    m_lastState = state;
    state = optimize(state, getState().angle);
    m_lastStateOptimized = state;
    driveMotor.set(
      state.speedMetersPerSecond //* DriveConstants.kMaxSpeedMetersPerSecond
    );
    turningMotor.set(
      m_lastPIDOutput =
        turningPidController.calculate(
          getAbsoluteEncoderRad(),
          state.angle.getRadians()
        )
    );
  }

  public SwerveModuleState getLastState() {
    return m_lastState;
  }

  public double getLastStateAngle() {
    return m_lastState.angle.getRadians();
  }

  public double getLastStateSpeed() {
    return m_lastState.speedMetersPerSecond;
  }

  public SwerveModuleState getLastStateOptimized() {
    return m_lastStateOptimized;
  }

  public double getLastStateAngleOptimized() {
    return m_lastStateOptimized.angle.getRadians();
  }

  public double getLastStateSpeedOptimized() {
    return m_lastStateOptimized.speedMetersPerSecond;
  }

  public double getLastPIDOutput() {
    return m_lastPIDOutput;
  }

  public double getLastError() {
    return getLastStateAngle() - getState().angle.getRadians();
  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public double getDrivePosition() {
    return driveMotor.getEncoder().getPosition();
  } // Returns position of drive encoder in meters traveled

  public double getTurningPosition() {
    return turningMotor.getEncoder().getPosition();
  } // Returns position of turning encoder in radians

  public double getDriveVelocity() {
    return driveMotor.getEncoder().getVelocity();
  } // Returns velocity of drive encoder in meters per second

  public double getTurningVelocity() {
    return turningMotor.getEncoder().getVelocity();
  } // Returns velocity of drive encoder in radians per second

  //the 4 methods above go through the conversion factor defined above in relative encoder

  public double getAbsoluteEncoderRot() {
    return absoluteEncoder.getAbsolutePosition().getValue().in(Rotations);
  } // Returns position of absolute encoder in ROTATIONS (not degrees) :(

  public double getAbsoluteEncoderRad() {
    return absoluteEncoder.getAbsolutePosition().getValue().in(Radians);
  } // Returns position of absolute encoder in radians

  public double getAbsoluteEncoderDeg() {
    return absoluteEncoder.getAbsolutePosition().getValue().in(Degrees);
  } // Returns position of absolute encoder in degrees

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      new Rotation2d(getAbsoluteEncoderRad())
    );
  } // Returns the above info in the form of a SwerveModuleState

  public static SwerveModuleState optimize(
    SwerveModuleState desiredState,
    Rotation2d currentAngle
  ) {
    var delta = desiredState.angle.minus(currentAngle);

    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
        -desiredState.speedMetersPerSecond,
        desiredState.angle.plus(Rotation2d.fromDegrees(180.0))
      );
    } else {
      return new SwerveModuleState(
        desiredState.speedMetersPerSecond,
        desiredState.angle
      );
    }
  }
}
