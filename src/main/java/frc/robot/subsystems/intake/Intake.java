package frc.robot.subsystems.intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Lights;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
	private final DigitalInput sensor;

	private GenericEntry noteIndicator, sensorOutput, sensorOverride;
	private Timer timer;
	private boolean sensorDisabled;

	/*
	 * Initialize all components and one-time logic to be completed on boot-up here
	 */
	public Intake(IntakeIO io) {
		this.io = io;
		sensor = new DigitalInput(IntakeConstants.kSensorPort);

		timer = new Timer();

		noteIndicator = Shuffleboard.getTab("Main").add("Note collected", Lights.getInstance().hasNote)
				.withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0).withSize(3, 3).getEntry();
		sensorOutput = Shuffleboard.getTab("Main").add("Sensor output", false).withWidget(BuiltInWidgets.kBooleanBox)
				.withPosition(0, 3).withSize(1, 1).getEntry();
		sensorOverride = Shuffleboard.getTab("Main").add("Sensor override", false)
				.withWidget(BuiltInWidgets.kBooleanBox).withPosition(1, 3).withSize(2, 1).getEntry();
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);

		Lights.getInstance().hasNote = !sensorDisabled && sensor.get();

		Logger.recordOutput("HoldingNote", Lights.getInstance().hasNote);
		Logger.recordOutput("Intake/NoteSensorOutput", sensor.get());
		Logger.recordOutput("Intake/NoteSensorDisabled", sensorDisabled);

		noteIndicator.setBoolean(Lights.getInstance().hasNote);
		sensorOutput.setBoolean(sensor.get());
		sensorOverride.setBoolean(sensorDisabled);

		if (timer.get() > 0.5) {
			RobotContainer.getDriverJoystick().setRumble(RumbleType.kBothRumble, 0);
			RobotContainer.getOperatorJoystick().setRumble(RumbleType.kBothRumble, 0);
			timer.stop();
		}
	}

	public void collectionIndicator() {
		RobotContainer.getDriverJoystick().setRumble(RumbleType.kBothRumble, 0.05);
		RobotContainer.getOperatorJoystick().setRumble(RumbleType.kBothRumble, 0.05);
		timer.reset();
		timer.start();
	}

	public void toggleSensorDisabled() {
		sensorDisabled = !sensorDisabled;
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
