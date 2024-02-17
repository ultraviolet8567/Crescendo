package frc.robot.subsystems.intake;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Lights;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

	private boolean notePreviouslyDetected = false;
	private int noteDetections = 0;
	private boolean noteCollected = false;

	private ColorSensorV3 sensor;
	private ColorMatch matcher;

	private double[] detectedColor;

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Intake(IntakeIO io) {
		this.io = io;

		sensor = new ColorSensorV3(I2C.Port.kOnboard);
		matcher = new ColorMatch();

		matcher.addColorMatch(Constants.kNoteColor);
		matcher.setConfidenceThreshold(Constants.kColorConfidenceThreshold);
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);

		// If the sensor sees orange, we have a note in the system
		ColorMatchResult result = matcher.matchColor(sensor.getColor());
		Lights.getInstance().hasNote = (result != null);

		// Record when note passes sensor
		if (!notePreviouslyDetected && Lights.getInstance().hasNote)
			io.stop();

		// If the note has passed the sensor twice, note has reached storing position
		// if (noteDetections >= 2) {
		// io.stop();
		// noteDetections = 0;
		// noteCollected = true;
		// }

		detectedColor = new double[]{sensor.getColor().red, sensor.getColor().green, sensor.getColor().blue};

		notePreviouslyDetected = Lights.getInstance().hasNote;
		Logger.recordOutput("Note Intaked", Lights.getInstance().hasNote);
		Logger.recordOutput("Intake/DetectedColor", detectedColor);
		Logger.recordOutput("Intake/NoteCollected", noteCollected);
	}

	/* Define all subsystem-specific methods and enums here */
	public void pickup() {
		if (!Lights.getInstance().hasNote) {
			io.setInputVoltage(IntakeConstants.kIntakeVoltage.get());
		}
	}

	public void drop() {
		io.setInputVoltage(-IntakeConstants.kIntakeVoltage.get());
	}

	public void runIndexer() {
		io.setInputVoltage(IntakeConstants.kIntakeVoltage.get());
	}

	public void stop() {
		io.stop();
	}
}
