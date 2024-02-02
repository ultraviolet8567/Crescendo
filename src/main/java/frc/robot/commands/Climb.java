package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class Climb extends Command {
	private Climber climb;
	private String direction;

	public Climb(Climber climb, String direction) {
		this.climb = climb;
		this.direction = direction;
	}

	@Override
	public void execute() {
		climb.startClimb(direction);
	}

	@Override
	public void end(boolean interrupted) {
		climb.stopClimb();
	}
}
