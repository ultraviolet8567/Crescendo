package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class RetractClimber extends Command {
	private Climber climber;
	private String side;

	public RetractClimber(Climber climber, String side) {
		this.climber = climber;
		this.side = side;
	}

	@Override
	public void execute() {
		if (side.equals("Right")) {
			climber.retractRight();
		} else if (side.equals("Left")) {
			climber.retractLeft();
		} else {
			climber.stop();
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (side.equals("Right")) {
			climber.stopRight();
		} else if (side.equals("Left")) {
			climber.stopLeft();
		} else {
			climber.stop();
		}
	}
}
