package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	/*
	 * Declare components of subsystem here (motor controllers, encoders, sensors,
	 * etc.)
	 */
	private final ShooterIO io;
	private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

	Intake intake;
	public double targetVel;

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */

	public Shooter(ShooterIO io, Intake intake) {
		this.io = io;
		this.intake = intake;
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
	public void shoot(double vel) {
		double voltage = 0;

		// Comment out if statement if not testing with intake
		if (intake.hasNote) {
			intake.hasNote = false;
			voltage = calculateVoltage(vel);
		}
		io.setInputVoltage(voltage, voltage);
	}

	public double calculateVoltage(double targetVelocity) {
		return io.calculateShooterTopVelocity(targetVelocity);
	}

	public void stop() {
		System.out.println("stopping");
		targetVel = 0.0;
		io.setInputVoltage(0.0, 0.0);
	}
}
