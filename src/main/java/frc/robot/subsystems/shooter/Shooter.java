package frc.robot.subsystems.shooter;

import static frc.robot.Constants.GainsConstants.shooterGains;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LoggedTunableNumber;
import java.util.Timer;
import java.util.TimerTask;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", shooterGains.kP());
	private static final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", shooterGains.kI());
	private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", shooterGains.kD());
	private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", shooterGains.ffkS());
	private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", shooterGains.ffkV());

	private final Arm arm;
	private final Intake intake;
	private final ShooterIO io;
	private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
	Timer timer = new Timer();

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Shooter(ShooterIO io, Arm arm, Intake intake) {
		this.io = io;
		this.arm = arm;
		this.intake = intake;
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
		// 87% buffer if the shooter flywheels never actually reach their target
		// velocity
		if (inputs.topVelocityRPM >= 0.87 * targetVel && inputs.bottomVelocityRPM >= 0.87 * targetVel) {
			intake.runIndexer();
			if (!intake.noteDetected) {
				timer.schedule(new TimerTask() {
					@Override
					public void run() {
						io.stop();
					}
				}, 1000);
			}
		}

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
