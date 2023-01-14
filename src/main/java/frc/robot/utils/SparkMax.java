package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

public class SparkMax extends CANSparkMax {
  public static final int PID_SLOT = 0;
  private static final int ALT_ENCODER_CPR = 8192;

  /**
   * Create a Spark Max object that is unit-testing friendly
   * @param deviceID
   * @param motorType
   */
  public SparkMax(int deviceID, MotorType motorType) {
    super(deviceID, motorType);
  }

  /**
   * Set motor output value
   * @param value Value to set
   * @param ctrl Desired control mode
   */
  public void set(double value, ControlType ctrl) {
    getPIDController().setReference(value, ctrl);
    set(value);
  }

  /**
   * Set motor output value with arbitrary feed forward
   * @param value Value to set
   * @param ctrl Desired control mode
   * @param arbFeedforward Feed forward value
   * @param arbFFUnits Feed forward units
   */
  public void set(double value, ControlType ctrl, double arbFeedforward, SparkMaxPIDController.ArbFFUnits arbFFUnits) {
    getPIDController().setReference(value, ctrl, PID_SLOT, arbFeedforward, arbFFUnits);
  }

  /**
   * Get the position of the motor. This returns the native units of 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   *
   * @return Number of rotations of the motor
   */
  public double getEncoderPosition() {
    return getEncoder().getPosition();
  }

  /**
   * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   *
   * @return Number the RPM of the motor
   */
  public double getEncoderVelocity() {
    return getEncoder().getVelocity();
  }

  /**
   * Returns an object for interfacing with the REV through bore encoder connected to the alternate encoder
   * mode data port pins. These are defined as:
   *
   * <ul>
   *   <li>Pin 4 (Forward Limit Switch): Index
   *   <li>Pin 6 (Multi-function): Encoder A
   *   <li>Pin 8 (Reverse Limit Switch): Encoder B
   * </ul>
   *
   * <p>This call will disable support for the limit switch inputs.
   *
   * @return An object for interfacing with a quadrature encoder connected to the alternate encoder
   *     mode data port pins
   */
  public RelativeEncoder getAlternateEncoder() {
    return getAlternateEncoder(Type.kQuadrature, ALT_ENCODER_CPR);
  }

  /**
   * Get the position of the motor. This returns the native units of 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   *
   * @return Number of rotations of the motor
   */
  public double getAlternateEncoderPosition() {
    return getAlternateEncoder(Type.kQuadrature, ALT_ENCODER_CPR).getPosition();
  }

  /**
   * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   *
   * @return Number the RPM of the motor
   */
  public double getAlternateEncoderVelocity() {
    return getAlternateEncoder(Type.kQuadrature, ALT_ENCODER_CPR).getVelocity();
  }

  /**
   * Reset NEO built-in encoder
   */
  public void resetEncoder() {
    getEncoder().setPosition(0.0);
  }

  /**
   * Reset external through bore encoder
   */
  public void resetAlternateEncoder() {
    getAlternateEncoder(Type.kQuadrature, ALT_ENCODER_CPR).setPosition(0.0);
  }
}
