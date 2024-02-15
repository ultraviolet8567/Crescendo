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
	}

	default void updateInputs(ArmIOInputs inputs) {
	}

	default void update() {
	}

	default double getPositionRads() {
		return 0.0;
	}

	default void setPosition(double setpoint) {
	}

	default void setInputVoltage(double volts) {
	}

	default void getPreset() {
	}

	default boolean armPastFrontLimit() {
		return true;
	}

	default boolean armPastBackLimit() {
		return true;
	}

	default boolean armWithinRange() {
		return false;
	}

	default void setBrakeMode() {
	}

	default void stop() {
	}

	default void setGains(double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {
	}

	default void resetAbsoluteEncoders() {
	}

	default void resetPIDControllers() {
	}
}
