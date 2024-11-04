package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.arm.Arm;
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
		double threshold = (getTargetVelocity() < 800)
				? ShooterConstants.kVelocityThresholdLow
				: ShooterConstants.kVelocityThreshold;

		return inputs.topVelocityRPM >= threshold * inputs.topTargetVelocityRPM
				&& inputs.bottomVelocityRPM >= threshold * inputs.bottomTargetVelocityRPM;
	}

	public double getTargetVelocity() {
		double vel = switch (arm.getArmMode()) {
			case SPEAKERFRONT -> ShooterConstants.kSpeakerFrontRPM.get();
			case SPEAKERANGLE -> ShooterConstants.kSpeakerAngleRPM.get();
			case SPEAKERSTAGE -> ShooterConstants.kSpeakerStageRPM.get();
			case AMP -> ShooterConstants.kAmpRPM.get();
			case TRAP -> ShooterConstants.kTrapRPM.get();
			default -> ShooterConstants.kIdleRPM.get();
		};

		if (Lights.getInstance().isDemo && vel >= 800) {
			return ShooterConstants.shooterDemoScaleFactor * vel;
		} else {
			return vel;
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
