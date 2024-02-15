package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
	@AutoLog
	class ShooterIOInputs {
		public double topVelocityRadPerSec = 0.0;
		public double topPositionRads = 0.0;
		public double topAppliedVoltage = 0.0;
		public double topCurrentAmps = 0.0;

		public double bottomVelocityRadPerSec = 0.0;
		public double bottomPositionRads = 0.0;
		public double bottomAppliedVoltage = 0.0;
		public double bottomCurrentAmps = 0.0;
	}

	default double getTopShooterVelocity() {
		return 0.0;
	}

	default double getBottomShooterVelocity() {
		return 0.0;
	}

	default double calculateShooterTopVelocity(double topVel) {
		return 0.0;
	}

	default double calculateShooterBottomVelocity(double bottomVel) {
		return 0.0;
	}

	default void updateInputs(ShooterIOInputs inputs) {
	}

	default void setTopInputVoltage(double volts) {
	}

	default void setBottomInputVoltage(double volts) {
	}

	default void setInputVoltage(double topVolts, double bottomVolts) {
	}

	default void stop() {
	}
}