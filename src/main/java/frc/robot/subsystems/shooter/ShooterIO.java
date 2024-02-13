package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
	@AutoLog
	class ShooterIOInputs {
		public double topVelocityRadPerSec = 0.0;
		public double topAppliedVoltage = 0.0;
		public double topCurrentAmps = 0.0;

		public double bottomVelocityRadPerSec = 0.0;
		public double bottomAppliedVoltage = 0.0;
		public double bottomCurrentAmps = 0.0;
	}

	default void updateInputs(ShooterIOInputs inputs) {
	}

	default void setTopInputVoltage(double volts) {
	}

	default void setBottomInputVoltage(double volts) {
	}

	default void setInputVoltage(double topVolts, double bottomVolts) {
	}

	default void setVelocity(double topTargetVel, double bottomTargetVel) {
	}

	default void stop() {
	}

	public default void setBrakeMode(boolean brake) {
	}
}
