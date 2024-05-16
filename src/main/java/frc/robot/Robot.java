package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.RobotState;
import frc.robot.util.VirtualSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
	private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert

	private final Timer canErrorTimer = new Timer();
	private final Timer canInitialErrorTimer = new Timer();
	private double autoStart;
	private boolean autoMessagePrinted;
	private double autoElapsedTime = 0.0;
	private double teleStart;
	private double teleElapsedTime = 0.0;

	private Command autonomousCommand;
	private RobotContainer robotContainer;

	private GenericEntry matchTimer;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Create lights
		Lights.getInstance();

		// Record metadata
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		switch (BuildConstants.DIRTY) {
			case 0 -> {
				Logger.recordMetadata("GitDirty", "All changes committed");
			}
			case 1 -> {
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
			}
			default -> {
				Logger.recordMetadata("GitDirty", "Unknown");
			}
		}

		// Set up data receivers & replay source
		switch (Constants.currentMode) {
			case REAL -> {
				Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
				Logger.addDataReceiver(new NT4Publisher());
			}
			case DEMO -> {
				// Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
				Logger.addDataReceiver(new NT4Publisher());
			}
			case SIM -> {
				Logger.addDataReceiver(new NT4Publisher());
			}
			case REPLAY -> {
				setUseTiming(false);
				String logpath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logpath));
				Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logpath, "_sim")));
			}
		}

		// Start AdvantageKit logger
		Logger.start();
		System.out.println("[Init] Starting AdvantageKit");

		// Log active commands
		Map<String, Integer> commandCounts = new HashMap<>();
		BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
			String name = command.getName();
			int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
			commandCounts.put(name, count);
			Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
			Logger.recordOutput("CommandsAll/" + name, count > 0);
		};
		CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
			logCommandFunction.accept(command, true);
		});
		CommandScheduler.getInstance().onCommandFinish((Command command) -> {
			logCommandFunction.accept(command, false);
		});
		CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
			logCommandFunction.accept(command, false);
		});

		// Reset alert timers
		canErrorTimer.restart();
		canInitialErrorTimer.restart();

		// Instantiate the RobotContainer
		RobotController.setBrownoutVoltage(6.0);
		robotContainer = new RobotContainer();
		System.out.println("[Init] Instantiating RobotContainer");

		// Post match timer to Shuffleboard
		matchTimer = Shuffleboard.getTab("Main").add("Time remaining", 0).withWidget(BuiltInWidgets.kNumberBar)
				.withProperties(Map.of("min", 0, "max", 135)).withPosition(0, 2).withSize(2, 1).getEntry();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. This runs after the
	 * mode specific periodic functions, but before LiveWindow and SmartDashboard
	 * integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		VirtualSubsystem.periodicAll();
		CommandScheduler.getInstance().run();

		// Check CAN status
		var canStatus = RobotController.getCANStatus();
		if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
			canErrorTimer.restart();
		}
		if (!canErrorTimer.hasElapsed(canErrorTimeThreshold)) {
			System.out.println("CAN errors detected, robot may not be controllable.");
			Logger.recordOutput("CANError", true);
		} else {
			Logger.recordOutput("CANError", false);
		}

		// Print auto duration
		if (autonomousCommand != null) {
			if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
				if (DriverStation.isAutonomousEnabled()) {
					System.out.printf("*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
				} else {
					System.out.printf("*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
				}
				autoMessagePrinted = true;
				Lights.getInstance().autoFinished = true;
				Lights.getInstance().autoFinishedTime = Timer.getFPGATimestamp();
			}
		}
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		Lights.getInstance().state = RobotState.DISABLED;
	}

	@Override
	public void disabledPeriodic() {
	}

	/** This function is called once each time the robot enters Autonomous mode. */
	@Override
	public void autonomousInit() {
		autoStart = Timer.getFPGATimestamp();

		// Set initial gyro yaw based on auto command
		// robotContainer.setInitialGyroYaw();

		// Set state to auto
		Lights.getInstance().state = RobotState.AUTO;

		// Run autonomous command
		autonomousCommand = robotContainer.getAutonomousCommand();
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}

	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		autoElapsedTime = Timer.getFPGATimestamp() - autoStart;
		matchTimer.setDouble(15.3 - autoElapsedTime); // Autonomous period is reliably 0.3s more than the nominal 15s
	}

	/** This function is called once each time the robot enters Teleop mode. */
	@Override
	public void teleopInit() {
		teleStart = Timer.getFPGATimestamp();

		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		// Set state to teleop
		Lights.getInstance().state = RobotState.TELEOP;
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		teleElapsedTime = Timer.getFPGATimestamp() - teleStart;
		matchTimer.setDouble(135 - teleElapsedTime);
	}

	/** This function is called once each time the robot enters Test mode. */
	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the simulation initializes */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}
