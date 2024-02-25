package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.RobotState;
import frc.robot.util.VirtualSubsystem;
import java.util.Map;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;

	private static GenericEntry timer;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		switch (BuildConstants.DIRTY) {
			case 0 :
				Logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1 :
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default :
				Logger.recordMetadata("GitDirty", "Unknown");
				break;
		}

		switch (Constants.currentMode) {
			case REAL :
				// Logger.addDataReceiver(new WPILOGWriter());
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case SIM :
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case REPLAY :
				setUseTiming(false);
				// String logpath = LogFileUtil.findReplayLog();
				// Logger.setReplaySource(new WPILOGReader(logpath));
				// Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logpath,
				// "_sim")));
				break;
		}

		System.out.println("[Init] Starting AdvantageKit");
		Logger.start();

		// Instantiate the RobotContainer
		System.out.println("[Init] Instantiating RobotContainer");
		m_robotContainer = new RobotContainer();

		Lights.getInstance();

		timer = Shuffleboard.getTab("Main").add("Time remaining", 0).withWidget(BuiltInWidgets.kNumberBar)
				.withProperties(Map.of("min", 0, "max", 135)).withPosition(4, 0).withSize(2, 1).getEntry();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics that you want ran during disabled, autonomous, teleoperated
	 * and test. This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		VirtualSubsystem.periodicAll();
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();

		// Update Shuffleboard
		timer.setDouble(DriverStation.getMatchTime());
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		Lights.getInstance().state = RobotState.DISABLED;
	}

	@Override
	public void disabledPeriodic() {
	}

	// uncomment below code if autonomous is ready to be developed
	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}

		// Set state to auto
		Lights.getInstance().state = RobotState.AUTO;
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		// Set state to teleop
		Lights.getInstance().state = RobotState.TELEOP;
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}
