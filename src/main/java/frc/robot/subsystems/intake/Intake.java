package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	/*
	 * Declare components of subsystem here (motor controllers, encoders, sensors,
	 * etc.)
	 */
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Intake(IntakeIO io) {
		this.io = io;
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		// Logger.recordOutput("Setpoints/Intake", intake.get());
		// Logger.recordOutput("Measured/Intake", intake.getEncoder().getVoltage());
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);
	}

	/* Define all subsystem-specific methods and enums here */

	public void pickup() {
		io.setInputVoltage(ArmConstants.intakeVoltage.get());
	}

	public void drop() {
		io.setInputVoltage(-ArmConstants.intakeVoltage.get());
	}

	public void stop() {
		io.stop();
	}
}
