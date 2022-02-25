// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivers;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;

/** Add your docs here. */
public class TurningEncoder implements Sendable {
  private final AnalogInput m_analogInput;
  private double m_angleOffset;
  private String m_prefName;
  private boolean m_reverse = false;

  public TurningEncoder(int channel) {
    this(new AnalogInput(channel));
  }

  /**
   * Construct a new TurningEncoder attached to a specific AnalogInput.
   *
   * @param analogInput the analog input to attach to
   */
  public TurningEncoder(AnalogInput analogInput) {
    m_analogInput = analogInput;
    init();
  }

  private void init() {
    m_prefName = String.format("TurningEncoderOffset(%01d)", m_analogInput.getChannel());
    m_angleOffset = Preferences.getDouble(m_prefName, 0);

    SendableRegistry.addLW(this, "TurningEncoder", m_analogInput.getChannel());
  }

  public double getAngleDeg() {
    double angle = ((m_analogInput.getVoltage() / RobotController.getVoltage5V()) * 360.0) - m_angleOffset;
    if (angle > 180)
      angle -= 360;
    if (angle < -180)
      angle += 360;
    if (m_reverse)
      angle = -angle;
    return -angle;
  }

  public double getAngleRad() {
    return Math.toRadians(getAngleDeg());
  }

  public void setReverseDirection(boolean reverse) {
    m_reverse = reverse;
  }

  public void reset() {
    m_angleOffset = (m_analogInput.getVoltage() / RobotController.getVoltage5V()) * 360.0;
    Preferences.setDouble(m_prefName, m_angleOffset);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("TurningEncoder");
    builder.addDoubleProperty("Angle(deg)", this::getAngleDeg, null);
  }

}
