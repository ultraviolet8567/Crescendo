package frc.robot.subsystems.shooter;

import static frc.robot.Constants.GainsConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	private final Arm arm;
	private final ShooterIO io;
	private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
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
	}

	/* Define all subsystem-specific methods and enums here */
	public void shoot() {
		double targetVel = getTargetVelocity();
		io.setVelocity(targetVel, targetVel);
	}

	public boolean atVelocity() {
		return inputs.topVelocityRPM >= ShooterConstants.kVelocityThreshold * getTargetVelocity()
				&& inputs.bottomVelocityRPM >= ShooterConstants.kVelocityThreshold * getTargetVelocity();
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
