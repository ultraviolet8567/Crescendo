package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
	@AutoLog
	class ShooterIOInputs {
		public double topVelocityRPM = 0.0;
		public double topAppliedVoltage = 0.0;
		public double[] topCurrentAmps = new double[]{0.0};
		public double[] topTempCelsius = new double[]{0.0};

		public double bottomVelocityRPM = 0.0;
		public double bottomAppliedVoltage = 0.0;
		public double[] bottomCurrentAmps = new double[]{0.0};
		public double[] bottomTempCelsius = new double[]{0.0};

		public double topRotations = 0.0;
		public double bottomRotations = 0.0;
	}

	default void updateInputs(ShooterIOInputs inputs) {
	}

	/** Sets top axle motor voltage */
	default void setTopInputVoltage(double volts) {
	}

	/** Sets bottom axle motor voltage */
	default void setBottomInputVoltage(double volts) {
	}

	/** Sets motor voltage for both axles */
	default void setInputVoltage(double topVolts, double bottomVolts) {
		setTopInputVoltage(topVolts);
		setBottomInputVoltage(bottomVolts);
	}

	/** Stops the arm motors */
	default void stop() {
	}

	/** Sets the PID and feed-forward parameters */
	default void setGains(double kP, double kI, double kD, double ffkS, double ffkV) {
	}

	/** Sets the idle mode of the motor */
	default void setBrakeMode(boolean brake) {
	}

	/** Run flywheels at exact velocity */
	default void setVelocity(double topTargetVel, double bottomTargetVel) {
	}
}
