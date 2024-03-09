package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AutoChooser extends SubsystemBase {
	private static final ShuffleboardTab main = Shuffleboard.getTab("Main");

	private final SendableChooser<String> sideOfField;
	private final SendableChooser<String> noteNumber;
	private final SendableChooser<String> otherStuff;

	public AutoChooser() {
		// Post the selectors to the ShuffleBoard
		noteNumber = new SendableChooser<>();
		noteNumber.setDefaultOption("0 Note", "Do Nothing");
		noteNumber.addOption("1 Note", "1 Note");
		noteNumber.addOption("2 Note", "2 Note");
		noteNumber.addOption("3 Note", "3 Note");
		noteNumber.addOption("4 Note", "4 Note");
		noteNumber.addOption("5 Note", "5 Note");
		main.add("Number of Notes", noteNumber).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1)
				.withPosition(3, 0);

		sideOfField = new SendableChooser<>();
		sideOfField.setDefaultOption("Amp", "Amp Side");
		sideOfField.addOption("Source", "Source Side");
		sideOfField.addOption("Center", "Center");
		main.add("Side of Field", sideOfField).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1)
				.withPosition(3, 1);

		otherStuff = new SendableChooser<>();
		otherStuff.setDefaultOption("None", "");
		otherStuff.addOption("Rush", "Rush ");
		otherStuff.addOption("Inner", "Inner ");
		otherStuff.addOption("Out of Way", "Out Of Way ");
		otherStuff.addOption("Don't Move", "Don't Move ");
		main.add("Other variables", otherStuff).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1)
				.withPosition(3, 2);
	}

	public String getAutoCommand() {
		String routineName = noteNumber.getSelected() + " " + otherStuff.getSelected() + sideOfField.getSelected();
		Logger.recordOutput("Auto/Routine", routineName);

		if (noteNumber.getSelected().equals("Do Nothing")) {
			return "Do Nothing";
		} else {
			return routineName;
		}
	}
}
