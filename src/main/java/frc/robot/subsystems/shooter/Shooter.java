package frc.robot.subsystems.shooter;

import static frc.robot.Constants.GainsConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Top/kP", shooterTopGains.kP());
	private static final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/Top/kI", shooterTopGains.kI());
	private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Top/kD", shooterTopGains.kD());
	private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Top/kS", shooterTopGains.ffkS());
	private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/Top/kV", shooterTopGains.ffkV());

	private final Arm arm;
	private final ShooterIO io;
	private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

	private SysIdRoutine routine;

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

		LoggedTunableNumber.ifChanged(hashCode(), () -> io.setGains(kP.get(), kI.get(), kD.get(), kS.get(), kV.get()),
				kP, kI, kD, kS, kV);
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
			case SPEAKER :
				return ShooterConstants.kSpeakerRPM.get();
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
