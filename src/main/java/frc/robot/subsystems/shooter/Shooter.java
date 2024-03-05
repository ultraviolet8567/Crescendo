package frc.robot.subsystems.shooter;

import static frc.robot.Constants.GainsConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LoggedTunableNumber;
import java.util.Timer;
import java.util.TimerTask;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	private static final LoggedTunableNumber TopkP = new LoggedTunableNumber("Shooter/Top/kP", shooterTopGains.kP());
	private static final LoggedTunableNumber TopkI = new LoggedTunableNumber("Shooter/Top/kI", shooterTopGains.kI());
	private static final LoggedTunableNumber TopkD = new LoggedTunableNumber("Shooter/Top/kD", shooterTopGains.kD());
	private static final LoggedTunableNumber TopkS = new LoggedTunableNumber("Shooter/Top/kS", shooterTopGains.ffkS());
	private static final LoggedTunableNumber TopkV = new LoggedTunableNumber("Shooter/Top/kV", shooterTopGains.ffkV());
	private static final LoggedTunableNumber BottomkP = new LoggedTunableNumber("Shooter/Bottom/kP", shooterBottomGains.kP());
	private static final LoggedTunableNumber BottomkI = new LoggedTunableNumber("Shooter/Bottom/kI", shooterBottomGains.kI());
	private static final LoggedTunableNumber BottomkD = new LoggedTunableNumber("Shooter/Bottom/kD", shooterBottomGains.kD());
	private static final LoggedTunableNumber BottomkS = new LoggedTunableNumber("Shooter/Bottom/kS", shooterBottomGains.ffkS());
	private static final LoggedTunableNumber BottomkV = new LoggedTunableNumber("Shooter/Bottom/kV", shooterBottomGains.ffkV());

	private final Arm arm;
	private final Intake intake;
	private final ShooterIO io;
	private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
	Timer timer = new Timer();

	private SysIdRoutine routine;

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

		LoggedTunableNumber.ifChanged(hashCode(), () -> io.setGains(TopkP.get(), TopkI.get(), TopkD.get(), TopkS.get(), TopkV.get(), BottomkP.get(), BottomkI.get(), BottomkD.get(), BottomkS.get(), BottomkV.get()),
				TopkP, TopkI, TopkD, TopkS, TopkV, BottomkP, BottomkI, BottomkD, BottomkS, BottomkV);
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
