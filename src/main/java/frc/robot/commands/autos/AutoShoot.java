package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShoot extends Command {
	private Shooter shooter;
	private Intake intake;
	private Timer timer;

	public AutoShoot(Shooter shooter, Intake intake) {
		this.shooter = shooter;
		this.intake = intake;
		timer = new Timer();

		addRequirements(shooter, intake);
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
		shooter.shoot();

		System.out.println("Shooting");
	}

	@Override
	public void execute() {
		if (shooter.atVelocity()) {
			intake.runIndexer();
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
		intake.stop();
	}

	@Override
	public boolean isFinished() {
		return shooter.atVelocity() && timer.get() > AutoConstants.kAutoShootTime;
	}
}
