package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class Shoot extends Command {
	private Shooter shooter;
	private Intake intake;
	private Odometry odometry;
	private Swerve swerve;
	private Arm arm;

	public Shoot(Shooter shooter, Intake intake, Swerve swerve, Arm arm, Odometry odometry) {
		this.shooter = shooter;
		this.intake = intake;
		this.odometry = odometry;
		this.swerve = swerve;
		this.arm = arm;

		addRequirements(shooter, intake);
	}

	@Override
	public void initialize() {
		shooter.shoot();
	}

	@Override
	public void execute() {
		if (shooter.atVelocity()) {
			intake.runIndexer();

			double targetRotZ = swerve.solveBodyRot(odometry.getPose(), FieldConstants.Speaker.centerSpeakerOpening);
			double targetRotY = arm.solveArmRot(odometry.getPose(), FieldConstants.Speaker.centerSpeakerOpening,
					ShooterConstants.kAutoShooterExitVel.get(), false);

			Logger.recordOutput("AutoTargeter/RotZ", targetRotZ);
			Logger.recordOutput("AutoTargeter/RotY", targetRotY);
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
		intake.stop();
	}
}
