package frc.robot.subsystems.shooter;

import static frc.robot.Constants.GainsConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LoggedTunableNumber;
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

	private double targetVel = ShooterConstants.kIdleRPM.get();

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

		Logger.recordOutput("Setpoints/ShooterTargetVelocity", targetVel);

		LoggedTunableNumber.ifChanged(hashCode(), () -> io.setGains(kP.get(), kI.get(), kD.get(), kS.get(), kV.get()),
				kP, kI, kD, kS, kV);
	}

	/* Define all subsystem-specific methods and enums here */
	public void shoot() {
		// if (!intake.noteDetected) {
		// return;
		// }

		targetVel = arm.getArmMode()[1];
		io.setVelocity(-targetVel, -targetVel);
	}

	public void stop() {
		io.stop();
	}
}
