package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drivers.TurningEncoder;

public class SwerveModuleMK2 {

  // TODO: Tune these PID values for your robot
  private static final double kDriveP = 15.0;
  private static final double kDriveI = 0.01;
  private static final double kDriveD = 0.1;
  private static final double kDriveF = 0.2;

  private static final double kAngleP = 0.2;
  private static final double kAngleI = 0.01;
  private static final double kAngleD = 0.0;
  private static final double kAngleF = 0;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  // private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
  //   kAngleP,
  //   kAngleI,
  //   kAngleD,
  //   new TrapezoidProfile.Constraints(
  //       SwerveDrivetrain.kMaxAngularSpeed,
  //       SwerveDrivetrain.kMaxAngularSpeed));
  private final PIDController m_turningPIDController = new PIDController(kAngleP, kAngleI, kAngleD);

  private TalonFX m_driveMotor;
  private TalonFX m_angleMotor;
  private TurningEncoder m_turningEncoder;
 
  public SwerveModuleMK2(TalonFX driveMotor, TalonFX angleMotor, TurningEncoder turningEncoder) {
    this.m_driveMotor = driveMotor;
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);

    this.m_angleMotor = angleMotor;
    this.m_turningEncoder = turningEncoder;
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.reset();
    m_turningPIDController.setTolerance(Math.toRadians(2.5));

    TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();

    driveTalonFXConfiguration.slot0.kP = kDriveP;
    driveTalonFXConfiguration.slot0.kI = kDriveI;
    driveTalonFXConfiguration.slot0.kD = kDriveD;
    driveTalonFXConfiguration.slot0.kF = kDriveF;

    driveMotor.configAllSettings(driveTalonFXConfiguration);
  }

  /**
   * Gets the relative rotational position of the module
   * @return The relative rotational position of the angle motor in Degrees
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_turningEncoder.getAngleDeg());
  }

  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * @param desiredState - A SwerveModuleState representing the desired new state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

    // Calculate turn output
    double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getAngleRad(), state.angle.getRadians());
    m_angleMotor.set(TalonFXControlMode.PercentOutput, turnOutput * 0.2);

    double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
    // m_driveMotor.set(TalonFXControlMode.PercentOutput, feetPerSecond / SwerveDrivetrain.kMaxSpeed);

    SmartDashboard.putNumber("Angle", m_turningEncoder.getAngleDeg());
    SmartDashboard.putNumber("SetPoint", state.angle.getDegrees());
    SmartDashboard.putNumber("TurnOutput", turnOutput);

  }

  /** Zeros all the SwerveModule encoders. */
  public void resetTurningEncoder() {
    m_turningEncoder.reset();
  }

  public void resetDrivingEncoder() {
    m_driveMotor.setSelectedSensorPosition(0.0);
  }

}
