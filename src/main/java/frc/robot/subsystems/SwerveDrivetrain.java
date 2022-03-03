// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

import frc.robot.drivers.TurningEncoder;

public class SwerveDrivetrain extends SubsystemBase {

  public static final double kMaxSpeed = Units.feetToMeters(13.6); // 13.6 feet per second
  public static final double kMaxAngularSpeed = 2 * Math.PI;       // 1 rotation per second

  /**
   * TODO: These are example values and will need to be adjusted for your robot!
   * Modules are in the order of -
   * Front Left
   * Front Right
   * Back Left
   * Back Right
   * 
   * Positive x values represent moving toward the front of the robot whereas
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(
      Units.inchesToMeters(11.5),
      Units.inchesToMeters(11.5)
    ),
    new Translation2d(
      Units.inchesToMeters(11.5),
      Units.inchesToMeters(-11.5)
    ),
    new Translation2d(
      Units.inchesToMeters(-11.5),
      Units.inchesToMeters(11.5)
    ),
    new Translation2d(
      Units.inchesToMeters(-11.5),
      Units.inchesToMeters(-11.5)
    )
  );

  private final Gyro m_gyro = new AHRS(SerialPort.Port.kMXP);

  private SwerveModuleMK2[] modules = new SwerveModuleMK2[] {
    new SwerveModuleMK2(new TalonFX(17), new TalonFX(18), new TurningEncoder(2)), // Front Left
    new SwerveModuleMK2(new TalonFX(11), new TalonFX(12), new TurningEncoder(3)), // Front Right
    new SwerveModuleMK2(new TalonFX(15), new TalonFX(16), new TurningEncoder(1)), // Back Left
    new SwerveModuleMK2(new TalonFX(13), new TalonFX(14), new TurningEncoder(0))  // Back Right
  };

  public SwerveDrivetrain() {
    m_gyro.reset(); 
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed);
    for (int i = 0; i < states.length; i++) {
      SwerveModuleMK2 module = modules[i];
      SwerveModuleState state = states[i];
      module.setDesiredState(state);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetTurningEncoders() {
    for (int i=0; i < 4; ++i) {
      modules[i].resetTurningEncoder();
    }
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }
  
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
    */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
    
}
