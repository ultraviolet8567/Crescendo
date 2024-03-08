package frc.robot.subsystems.intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Lights;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
	private final DigitalInput sensor;

	private GenericEntry noteIndicator;

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Intake(IntakeIO io) {
		this.io = io;
		sensor = new DigitalInput(IntakeConstants.kSensorPort);

		noteIndicator = Shuffleboard.getTab("Main").add("Note collected", Lights.getInstance().hasNote)
				.withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0).withSize(2, 2).getEntry();
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);

		Lights.getInstance().hasNote = false; // sensor.get();

		Logger.recordOutput("HoldingNote", Lights.getInstance().hasNote);
		Logger.recordOutput("SensorOutput", sensor.get());
		noteIndicator.setBoolean(Lights.getInstance().hasNote);
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
