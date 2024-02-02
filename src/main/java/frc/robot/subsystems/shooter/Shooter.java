package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	/*
	 * Declare components of subsystem here (motor controllers, encoders, sensors,
	 * etc.)
	 */
	private final ShooterIO io;
	private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */

	// Add shooter ports!!!!!

	public Shooter(ShooterIO io) {
		this.io = io;
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		// Logger.recordOutput("Setpoints/Intake", shooterTop.get());
		// Logger.recordOutput("Measured/Intake",
		// shooterTop.getEncoder().getVelocity());
		// Logger.recordOutput("Setpoints/Intake", shooterBottom.get());
		// Logger.recordOutput("Measured/Intake",
		// shooterBottom.getEncoder().getVelocity());

		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);
	}

	/* Define all subsystem-specific methods and enums here */
	public void shoot(double topVoltage, double bottomVoltage) {
		io.setInputVoltage(topVoltage, bottomVoltage);
	}

	public void stop() {
		io.setInputVoltage(0.0, 0.0);
	}
}
