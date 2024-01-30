package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class YawAlign extends Command {
	private final Swerve swerve;
	private final Vision vision;
	private ShuffleboardTab tab = Shuffleboard.getTab("Horrible, horrible Shuffleboard");
	private GenericEntry alliance = tab.add("Alliance", false).getEntry();

	public YawAlign(Swerve swerve, Vision vision) {
		this.swerve = swerve;
		this.vision = vision;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		turn(vision.getDistToAlign(alliance.get().getBoolean(), vision.getTargets()));
	}

	@Override
	public void end(boolean interrupted) {
	}

	public void turn(Rotation2d angle) {
		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(1.0, 0, 0, angle);
		SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
		swerve.setModuleStates(moduleStates);
	}
}
