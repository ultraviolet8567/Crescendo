package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
	@AutoLog
	class ArmIOInputs {
		public double velocityRadPerSec = 0.0;
		public double positionRads = 0.0;
		public double appliedVoltage = 0.0;
		public double[] currentAmps = new double[]{};
		public double[] tempCelsius = new double[]{};
		public boolean withinRange = false;
	}

	default void updateInputs(ArmIOInputs inputs) {
	}

	/** Sets motor voltage */
	default void setInputVoltage(double volts) {
	}

	/** Stops the arm motors */
	default void stop() {
	}

	/** Sets the PID and feed-forward parameters */
	default void setGains(double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {
	}

	/** Sets the idle mode of the motor */
	default void setBrakeMode(boolean brake) {
	}

	default void update() {
	}

	/** Returns the current position of the arm in radians */
	default double getPositionRads() {
		return 0.0;
	}

	/** Move arm to exact angle */
	default void setPosition(double setpoint) {
	}

	/** Checks if arm has gone past its front limit */
	default boolean armPastFrontLimit() {
		return true;
	}

	/** Checks if arm has gone past its back limit */
	default boolean armPastBackLimit() {
		return true;
	}

	/** Checks if arm is within its safe operable range */
	default boolean armWithinRange() {
		return !armPastFrontLimit() && !armPastBackLimit();
	}

	/** Resets the encoder reading to 0 */
	default void resetAbsoluteEncoders() {
	}

	/** Resets the PID controller to start anew */
	default void resetPIDControllers() {
	}
}
