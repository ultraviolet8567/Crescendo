package frc.robot.subsystems.arm;

import static frc.robot.Constants.GainsConstants.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
	private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", armGains.kP());
	private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", armGains.kI());
	private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", armGains.kD());
	private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", armGains.ffkS());
	private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", armGains.ffkV());
	private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/kA", armGains.ffkA());
	private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", armGains.ffkG());

	private final ArmIO io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private ArmMode armMode;

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Arm(ArmIO io) {
		this.io = io;
		armMode = ArmMode.MANUAL;
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		io.update();

		Logger.processInputs("Arms", inputs);
		// Logger.recordOutput("Arm/Mode", armMode);

		LoggedTunableNumber.ifChanged(hashCode(),
				() -> io.setGains(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get()), kP, kI, kD, kS,
				kV, kA, kG);

		Logger.recordOutput("Arm/Mode", armMode);
	}

	/* Define all subsystem-specific methods and enums here */
	public void setTargetAngle(double targetAngle) {
		if (io.armWithinRange()) {
			io.setPosition(targetAngle);
		}
	}

	public void setTurnSpeed(double factor) {
		double voltage = factor * ArmConstants.kManualVoltage.get();
		if (io.armWithinRange()) {
			io.setInputVoltage(voltage);
		} else if (io.armPastBackLimit() && voltage <= 0) {
			io.setInputVoltage(voltage);
		} else if (io.armPastFrontLimit() && voltage >= 0) {
			io.setInputVoltage(voltage);
		} else {
			io.setInputVoltage(0.0);
		}
	}

	public void setArmMode(ArmMode mode) {
		armMode = mode;

		if (armMode == ArmMode.MANUAL) {
			resetPIDControllers();
		}
	}

	public ArmMode getArmMode() {
		return armMode;
	}

	public double getPresetAngle() {
		switch (armMode) {
			case SPEAKER :
				return ArmConstants.kSpeakerAngle;
			case AMP :
				return ArmConstants.kAmpAngle;
			case ROOMBA :
				return ArmConstants.kRoombaAngle;
			case TAXI :
				return ArmConstants.kTaxiAngle;
			case TRAP :
				return ArmConstants.kTrapAngle;
			default :
				return ArmConstants.kTaxiAngle;
		}
	}

	public Transform3d getDeltaY() {
		double y = ArmConstants.kArmLength * Math.sin(io.getPositionRads());
		return new Transform3d(0.0, y, 0.0, new Rotation3d());
	}

	public void stop() {
		io.stop();
	}

	public void resetAbsoluteEncoders() {
		io.resetAbsoluteEncoders();
	}

	public void resetPIDControllers() {
		io.resetPIDControllers();
	}

	public static enum ArmMode {
		/** Manual control */
		MANUAL,
		/** Intaking from ground */
		ROOMBA,
		/** Pointing at speaker */
		SPEAKER,
		/** Pointing at amp */
		AMP,
		/** Pointing at trap */
		TRAP,
		/** Stowed in taxi position */
		TAXI
	}

}
