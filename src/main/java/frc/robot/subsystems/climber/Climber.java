package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
	private final ClimberIO io;
	private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Climber(ClimberIO io) {
		this.io = io;
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Climber", inputs);
	}

	/* Define all subsystem-specific methods and enums here */
	public void startClimb(String direction) {
		if (direction.equals("extend")) {
			io.setInputVoltage(ClimberConstants.climbVoltage.get());
		} else {
			io.setInputVoltage(-ClimberConstants.climbVoltage.get());
		}
	}

	public void stopClimb() {
		io.stop();
	}
}
