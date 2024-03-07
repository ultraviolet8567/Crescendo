package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.GainsConstants.armGains;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
	private static final LoggedTunableNumber angle = new LoggedTunableNumber("Arm/Angle", -Math.PI / 2);

	private final ArmIO io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private ArmMode armMode;
	private SysIdRoutine routine;

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Arm(ArmIO io) {
		this.io = io;
		armMode = ArmMode.MANUAL;

		// routine = new SysIdRoutine(new
		// SysIdRoutine.Config(Volts.of(2).per(Seconds.of(1)), Volts.of(5),
		// Seconds.of(5)),
		// new SysIdRoutine.Mechanism((Measure<Voltage> volts) ->
		// io.setInputVoltage(volts.in(Volts)),
		// log -> log.motor("arm").voltage(Volts.of(inputs.appliedVoltage))
		// .angularPosition(Radians.of(inputs.positionRads))
		// .angularVelocity(RadiansPerSecond.of(inputs.velocityRadPerSec)),
		// this));
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Arms", inputs);

		Logger.recordOutput("Arm/Mode", armMode);
		Logger.recordOutput("Arm/Setpoint", getPresetAngle());

		// Check if the gains configuration has changed
		LoggedTunableNumber.ifChanged(hashCode(),
				() -> io.setGains(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get()), kP, kI, kD, kS,
				kV, kA, kG);

	}

	// Move arm to exact angle
	public void setPosition(double targetAngle) {
		if (io.armWithinRange()) {
			io.setPosition(targetAngle);
			Logger.recordOutput("Setpoints/Arm", targetAngle);
		}
	}

	// Manually control arm
	public void setTurnSpeed(double factor) {
		double voltage = factor * ArmConstants.kManualVoltage.get();

		// Make sure the arm isn't moving out of operable range
		if (io.armWithinRange() || io.armPastBackLimit() && voltage <= 0 || io.armPastFrontLimit() && voltage >= 0) {
			io.setInputVoltage(voltage);
		} else {
			io.stop();
		}
	}

	public ArmMode getArmMode() {
		return armMode;
	}

	public void setArmMode(ArmMode mode) {
		armMode = mode;
		if (armMode == ArmMode.MANUAL) {
			resetPIDControllers();
		}
	}

	public double getPresetAngle() {
		// return angle.get();
		switch (armMode) {
			case SPEAKERFRONT :
				return ArmConstants.kSpeakerFrontAngle;
			case SPEAKERANGLE :
				return ArmConstants.kSpeakerAngleAngle;
			case SPEAKERSTAGE :
				return ArmConstants.kSpeakerStageAngle;
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

	public boolean atSetpoint() {
		return Math.abs(getPresetAngle() - inputs.positionRads) < ArmConstants.kSetpointTolerance;
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
		/** Pointing at speaker directly in front of the subwoofer */
		SPEAKERFRONT,
		/** Pointing at speaker on the angled side of the subwoofer */
		SPEAKERANGLE,
		/** Pointing at speaker on the right side of the stage */
		SPEAKERSTAGE,
		/** Pointing at amp */
		AMP,
		/** Pointing at trap */
		TRAP,
		/** Stowed in taxi position */
		TAXI
	}

	public SysIdRoutine routine() {
		return routine;
	}
}
