package frc.robot.subsystems.intake;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

	public boolean hasNote = false;
	public boolean isPicking = false;
	public boolean isSeeingOrange = false;
	public boolean wasSeeingOrange = false;

	public int colorPasses = 0;

	private ColorSensorV3 sensor;
	private ColorMatch matcher;

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Intake(IntakeIO io) {
		this.io = io;
		sensor = new ColorSensorV3(ArmConstants.kArmColorSensorPort);
		matcher = new ColorMatch();

		matcher.addColorMatch(ArmConstants.kNoteColor);
		matcher.setConfidenceThreshold(ArmConstants.kColorConfidenceThreshold);
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);
		
		if (isPicking) {
			ColorMatchResult result = matcher.matchColor(sensor.getColor());

			if (result == null) {
				isSeeingOrange = false;
			}
			else {
				isSeeingOrange = true;
			}

			if (wasSeeingOrange && !isSeeingOrange) {
				colorPasses += 1;
			}

			if (colorPasses >= 2) {
				isPicking = false;
				hasNote = true;
				io.stop();
			}

			wasSeeingOrange = isSeeingOrange;
		}
	}

	/* Define all subsystem-specific methods and enums here */
	public void pickup() {
		if (!hasNote) {
			io.setInputVoltage(ArmConstants.kIntakeVoltage.get());
			isPicking = true;
			colorPasses = 0;
		}
	}

	public void drop() {
		io.setInputVoltage(-ArmConstants.kIntakeVoltage.get());
	}

	public void stop() {
		io.stop();
	}
}
