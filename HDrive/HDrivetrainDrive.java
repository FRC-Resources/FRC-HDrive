package frc.packages.HDrive;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class HDrivetrainDrive extends RobotDriveBase implements Sendable, AutoCloseable {
  private static int instances;

  private final MotorController m_leftMotor;
  private final MotorController m_rightMotor;
  private final MotorController m_lateralMotor;

  private boolean m_reported;

  /**
   * Construct a HDrive.
   *
   * <p>
   * To pass multiple motors per side, use a {@link
   * edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup}. If a motor needs to
   * be inverted, do
   * so before passing it in.
   *
   * @param leftMotor  Left motor.
   * @param rightMotor Right motor.
   * @param lateralMotor Lateral motor.
   */
  public HDrivetrainDrive(MotorController leftMotor, MotorController rightMotor, MotorController lateralMotor) {
    requireNonNullParam(leftMotor, "leftMotor", "HDrivetrainDrive");
    requireNonNullParam(rightMotor, "rightMotor", "HDrivetrainDrive");
    requireNonNullParam(lateralMotor, "lateralMotor", "HDrivetrainDrive");

    m_leftMotor = leftMotor;
    m_rightMotor = rightMotor;
    m_lateralMotor = lateralMotor;
    SendableRegistry.addChild(this, m_leftMotor);
    SendableRegistry.addChild(this, m_rightMotor);
    SendableRegistry.addChild(this, m_lateralMotor);
    instances++;
    SendableRegistry.addLW(this, "HDrivetrainDrive", instances);
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  /**
   * Arcade drive method for differential drive platform. The calculated values
   * will be squared to
   * decrease sensitivity at low speeds.
   *
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is
   *                  positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                  Counterclockwise is
   *                  positive.
   * @param lateralSpeed The robot's speed along the robots lateral axis [-1.0..1.0]. Right is positive.
   */
  public void arcadeDrive(double xSpeed, double zRotation, double lateralSpeed) {
    arcadeDrive(xSpeed, zRotation, lateralSpeed, true);
  }

  /**
   * Arcade drive method for differential drive platform.
   *
   * @param xSpeed       The robot's speed along the X axis [-1.0..1.0]. Forward
   *                     is positive.
   * @param zRotation    The robot's rotation rate around the Z axis [-1.0..1.0].
   *                     Counterclockwise is
   *                     positive.
   * @param lateralSpeed The robot's speed along the robots lateral axis [-1.0..1.0]. Right is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void arcadeDrive(double xSpeed, double zRotation, double lateralSpeed, boolean squareInputs) {
    if (!m_reported) {
      HAL.report(
          tResourceType.kResourceType_RobotDrive, tInstances.kRobotDrive2_DifferentialArcade, 2);
      m_reported = true;
    }

    xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);
    zRotation = MathUtil.applyDeadband(zRotation, m_deadband);

    var speeds = arcadeDriveIK(xSpeed, zRotation, lateralSpeed, squareInputs);

    m_leftMotor.set(speeds.left * m_maxOutput);
    m_rightMotor.set(speeds.right * m_maxOutput);
    m_lateralMotor.set(lateralSpeed * m_maxOutput/2);

    feed();
  }

  /**
   * Arcade drive inverse kinematics for differential drive platform.
   *
   * @param xSpeed       The robot's speed along the X axis [-1.0..1.0]. Forward
   *                     is positive.
   * @param zRotation    The robot's rotation rate around the Z axis [-1.0..1.0].
   *                     Counterclockwise is
   *                     positive.
   * @param lateralSpeed The robot's speed along the robots lateral axis [-1.0..1.0]. Right is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   * @return Wheel speeds [-1.0..1.0].
   */
  public static WheelSpeeds arcadeDriveIK(double xSpeed, double zRotation, double lateralSpeed, boolean squareInputs) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    lateralSpeed = MathUtil.clamp(lateralSpeed, -1.0, 1.0);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftSpeed = xSpeed - zRotation;
    double rightSpeed = xSpeed + zRotation;

    // Find the maximum possible value of (throttle + turn + lateral) along the vector
    // that the joystick is pointing, then desaturate the wheel speeds
    double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
    double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotation));
    if (greaterInput == 0.0) {
      return new WheelSpeeds(0.0, 0.0, 0.0);
    }
    double saturatedInput = (greaterInput + lesserInput) / greaterInput;
    leftSpeed /= saturatedInput;
    rightSpeed /= saturatedInput;
    lateralSpeed /= saturatedInput;

    return new WheelSpeeds(leftSpeed, rightSpeed, lateralSpeed);
  }

  /**
   * Tank drive method for differential drive platform. The calculated values will
   * be squared to
   * decrease sensitivity at low speeds.
   *
   * @param leftSpeed  The robot's left side speed along the X axis [-1.0..1.0].
   *                   Forward is positive.
   * @param rightSpeed The robot's right side speed along the X axis [-1.0..1.0].
   *                   Forward is
   *                   positive.
   * @param lateralSpeed The robot's speed along the Y axis [-1.0..1.0]. Right is positive
   */
  public void tankDrive(double leftSpeed, double rightSpeed, double lateralSpeed) {
    tankDrive(leftSpeed, rightSpeed, lateralSpeed, true);
  }

  /**
   * Tank drive inverse kinematics for differential drive platform.
   *
   * @param leftSpeed    The robot left side's speed along the X axis [-1.0..1.0].
   *                     Forward is positive.
   * @param rightSpeed   The robot right side's speed along the X axis
   *                     [-1.0..1.0]. Forward is
   *                     positive.
   * @param lateralSpeed The robot's speed along the Y axis [-1.0..1.0]. right is negative
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   * @return Wheel speeds [-1.0..1.0].
   */
  public static WheelSpeeds tankDriveIK(double leftSpeed, double rightSpeed, double lateralSpeed, boolean squareInputs) {
    leftSpeed = MathUtil.clamp(leftSpeed, -1.0, 1.0);
    rightSpeed = MathUtil.clamp(rightSpeed, -1.0, 1.0);
    lateralSpeed = MathUtil.clamp(lateralSpeed, -1.0, 1.0);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      leftSpeed = Math.copySign(leftSpeed * leftSpeed, leftSpeed);
      rightSpeed = Math.copySign(rightSpeed * rightSpeed, rightSpeed);
      lateralSpeed = Math.copySign(lateralSpeed * lateralSpeed, lateralSpeed);
    }

    return new WheelSpeeds(leftSpeed, rightSpeed, lateralSpeed);
  }

  /**
   * Tank drive method for differential drive platform.
   *
   * @param leftSpeed    The robot left side's speed along the X axis [-1.0..1.0].
   *                     Forward is positive.
   * @param rightSpeed   The robot right side's speed along the X axis
   *                     [-1.0..1.0]. Forward is
   *                     positive.
   * @param lateralSpeed The robot's speed along the Y axis [-1.0..1.0]. Right is positive
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void tankDrive(double leftSpeed, double rightSpeed, double lateralSpeed, boolean squareInputs) {
    if (!m_reported) {
      HAL.report(
          tResourceType.kResourceType_RobotDrive, tInstances.kRobotDrive2_DifferentialTank, 2);
      m_reported = true;
    }

    leftSpeed = MathUtil.applyDeadband(leftSpeed, m_deadband);
    rightSpeed = MathUtil.applyDeadband(rightSpeed, m_deadband);

    var speeds = tankDriveIK(leftSpeed, rightSpeed, lateralSpeed, squareInputs);

    m_leftMotor.set(speeds.left * m_maxOutput);
    m_rightMotor.set(speeds.right * m_maxOutput);
    m_lateralMotor.set(speeds.lateral * m_maxOutput);

    feed();
  }

  @Override
  public void stopMotor() {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
    m_lateralMotor.stopMotor();
    feed();
  }

  @Override
  public String getDescription() {
    return "HDrivetrainDrive";
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HDrivetrainDrive");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Left Motor Speed", m_leftMotor::get, m_leftMotor::set);
    builder.addDoubleProperty("Right Motor Speed", () -> m_rightMotor.get(), x -> m_rightMotor.set(x));
    builder.addDoubleProperty("Lateral Motor Speed", () -> m_lateralMotor.get(), x -> m_lateralMotor.set(x));
  }

  /**
   * Wheel speeds for a differential drive.
   *
   * <p>
   * Uses normalized voltage [-1.0..1.0].
   */
  @SuppressWarnings("MemberName")
  public static class WheelSpeeds {
    public double left;
    public double right;
    private double lateral;

    /** Constructs a WheelSpeeds with zeroes for left and right speeds. */
    public WheelSpeeds() {
    }

    /**
     * Constructs a WheelSpeeds.
     *
     * @param left  The left speed [-1.0..1.0].
     * @param right The right speed [-1.0..1.0].
     * @param lateral The lateral wheel speed [-1.0..1.0].
     */
    public WheelSpeeds(double left, double right, double lateral) {
      this.left = left;
      this.right = right;
      this.lateral = lateral;
    }
  }
}