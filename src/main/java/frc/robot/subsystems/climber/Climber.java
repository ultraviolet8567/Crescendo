package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Lights;
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

		if (climbed()) {
			Lights.getInstance().climbed = true;
		}
	}

	/* Define all subsystem-specific methods and enums here */
	public void extendLeft() {
		if (leftClimberWithinRange() || leftClimberGoingIntoRange(-1)) {
			io.setLeftInputVoltage(ClimberConstants.kClimbVoltage.get());
		} else {
			io.stopLeft();
		}
	}

	public void extendRight() {
		if (rightClimberWithinRange() || rightClimberGoingIntoRange(-1)) {
			io.setRightInputVoltage(ClimberConstants.kClimbVoltage.get());
		} else {
			io.stopRight();
		}
	}

	public void retractLeft() {
		if (leftClimberWithinRange() || leftClimberGoingIntoRange(1)) {
			io.setLeftInputVoltage(-ClimberConstants.kClimbVoltage.get());
		} else {
			io.stopLeft();
		}
	}

	public void retractRight() {
		if (rightClimberWithinRange() || rightClimberGoingIntoRange(1)) {
			io.setRightInputVoltage(-ClimberConstants.kClimbVoltage.get());
		} else {
			io.stopRight();
		}
	}

	public boolean climbed() {
		return false; // see if z position changes much? or if the robot tilts a lot use angle (x/y?)
	}

	public boolean leftClimberWithinRange() {
		return !io.leftClimberPastBackLimit() && !io.leftClimberPastFrontLimit();
	}

	public boolean rightClimberWithinRange() {
		return !io.rightClimberPastBackLimit() && !io.rightClimberPastFrontLimit();
	}

	public boolean leftClimberGoingIntoRange(double voltage) {
		return io.leftClimberPastBackLimit() && voltage >= 0 || io.leftClimberPastFrontLimit() && voltage <= 0;
	}

	public boolean rightClimberGoingIntoRange(double voltage) {
		return io.rightClimberPastBackLimit() && voltage >= 0 || io.rightClimberPastFrontLimit() && voltage <= 0;
	}

	public void stop() {
		io.stop();
	}

	public void stopLeft() {
		io.stopLeft();
	}

	public void stopRight() {
		io.stopRight();
	}
}
