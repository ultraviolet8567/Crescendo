package frc.robot.subsystems.intake;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Lights;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

	private GenericEntry noteIndicator;

	private boolean notePreviouslyDetected = false;
	private boolean noteDetected = false;

	private ColorSensorV3 sensor;
	private ColorMatch matcher;

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Intake(IntakeIO io) {
		this.io = io;

		// sensor = new ColorSensorV3(I2C.Port.kOnboard);
		// matcher = new ColorMatch();
		// matcher.addColorMatch(Constants.kNoteColor);
		// matcher.setConfidenceThreshold(Constants.kColorConfidenceThreshold);

		noteIndicator = Shuffleboard.getTab("Main").add("Note collected", Lights.getInstance().hasNote)
				.withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0).withSize(2, 2).getEntry();
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);

		noteIndicator.setBoolean(Lights.getInstance().hasNote);

		Logger.recordOutput("HoldingNote", Lights.getInstance().hasNote);
		// Logger.recordOutput("Intake/DetectedColor",
		// new double[]{sensor.getColor().red, sensor.getColor().green,
		// sensor.getColor().blue});
		// Logger.recordOutput("Intake/Proximity", sensor.getProximity());

		// If the sensor sees orange, we have a note in the system
		// ColorMatchResult result = matcher.matchColor(sensor.getColor());
		// noteDetected = (result != null);

		Lights.getInstance().hasNote = noteDetected;
		// if (!notePreviouslyDetected && noteDetected) {
		// Lights.getInstance().hasNote = true;
		// }

		// notePreviouslyDetected = noteDetected;
	}

	public void pickup() {
		if (!Lights.getInstance().hasNote) {
			io.setInputVoltage(IntakeConstants.kIntakeVoltage.get());
		}
	}

	public void drop() {
		io.setInputVoltage(-IntakeConstants.kIntakeVoltage.get());
	}

	public void runIndexer() {
		// Go at max speed to minimize drag on note
		io.setInputVoltage(12.0);
	}

	public void stop() {
		io.stop();
	}
}
