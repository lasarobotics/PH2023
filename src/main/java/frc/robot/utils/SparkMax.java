package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

public class SparkMax extends CANSparkMax {
  public static final int PID_SLOT = 0;

  /**
   * Create a Spark Max object that is unit-testing friendly
   * @param deviceID
   * @param motorType
   */
  public SparkMax(int deviceID, MotorType motorType) {
    super(deviceID, motorType);
  }

  public void set(double value, ControlType ctrl) {
    getPIDController().setReference(value, ctrl);
  }

  public void set(double value, ControlType ctrl, double arbFeedforward, SparkMaxPIDController.ArbFFUnits arbFFUnits) {
    getPIDController().setReference(value, ctrl, PID_SLOT, arbFeedforward, arbFFUnits);
  }
}
