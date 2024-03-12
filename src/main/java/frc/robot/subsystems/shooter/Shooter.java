package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	private final Arm arm;
	private final ShooterIO io;
	private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

	/*
	 * Initialize all components and one-time logic to be completed on boot-up here
	 */
	public Shooter(ShooterIO io, Arm arm) {
		this.io = io;
		this.arm = arm;
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);

		Logger.recordOutput("Shooter/TargetVelocity", getTargetVelocity());
		Logger.recordOutput("Shooter/AtVelocity", atVelocity());
	}

	public void shoot() {
		double targetVel = getTargetVelocity();
		io.setVelocity(targetVel, targetVel);
	}

	public void autoShoot(double rpm) {
		io.setVelocity(rpm, rpm);
	}

	public boolean atVelocity() {
		// Velocity threshold for Amp shots is lower because shot velocity requires less
		// precision
		double threshold = (arm.getArmMode() == ArmMode.AMP)
				? ShooterConstants.kVelocityThresholdAmp
				: ShooterConstants.kVelocityThreshold;

		return inputs.topVelocityRPM >= threshold * getTargetVelocity()
				&& inputs.bottomVelocityRPM >= threshold * getTargetVelocity();
	}

	public double getTargetVelocity() {
		switch (arm.getArmMode()) {
			case SPEAKERFRONT :
				return ShooterConstants.kSpeakerFrontRPM.get();
			case SPEAKERANGLE :
				return ShooterConstants.kSpeakerAngleRPM.get();
			case SPEAKERSTAGE :
				return ShooterConstants.kSpeakerStageRPM.get();
			case AMP :
				return ShooterConstants.kAmpRPM.get();
			case TRAP :
				return ShooterConstants.kTrapRPM.get();
			default :
				return ShooterConstants.kIdleRPM.get();
		}
	}

	public void stop() {
		io.stop();
	}
}
