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

	public void shoot(double scaleDown) {
		double targetVel = scaleDown * getTargetVelocity();
		io.setVelocity(targetVel, targetVel);
	}

	public boolean atVelocity() {
		// Velocity threshold for Amp shots is lower because shot velocity requires less
		// precision
		double threshold = (arm.getArmMode() == ArmMode.AMP)
				? ShooterConstants.kVelocityThresholdAmp
				: ShooterConstants.kVelocityThreshold;

		return inputs.topVelocityRPM >= threshold * inputs.topTargetVelocityRPM
				&& inputs.bottomVelocityRPM >= threshold * inputs.bottomTargetVelocityRPM;
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

// Goes in Arm:
/*
 * public double solveArmRot(double xDiff, double zDiff, double exitVel, boolean
 * isUpper) { //Desmos calculation to translate into code (do not uncomment): //
 * a_{1}=\arctan\left(\frac{-T.x+\sqrt{T.x^{2}-4\left(\frac{gT.x^{2}}{v^{2}}\
 * cdot\left(\frac{gT.x^{2}}{v^{2}}-T.y\right)\right)}}{2\cdot\frac{gT.x^{2}}{v^
 * {2}}}\right) // \frac{gT.x^{2}}{v^{2}}
 *
 * double gravity = -9.8; double var = Math.pow(xDiff, 2) * gravity /
 * Math.pow(exitVel, 2); double deriv = Math.pow(xDiff, 2) - 4 * (var * (var -
 * zDiff));
 *
 * if (isUpper) { return Math.atan((-xDiff + Math.sqrt(deriv)) / (2 * var)); }
 * else { return Math.atan((-xDiff - Math.sqrt(deriv)) / (2 * var)); }
 */
