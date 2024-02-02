package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {
	// CHECK IF GEARING AND JKGMETERSSQUARED ARE RIGHT
	private final SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(2), 0.1, 0.01, 0.58,
			ArmConstants.kMinArmAngle, ArmConstants.kMaxArmAngle, true, Math.PI / 2);
	private final Constraints armConstraints = new Constraints(ArmConstants.kMaxSpeed.get(),
			ArmConstants.kMaxAcceleration.get());
	private final ProfiledPIDController armPIDController = new ProfiledPIDController(ArmConstants.kP.get(),
			ArmConstants.kI.get(), ArmConstants.kD.get(), armConstraints);

	Mechanism2d arm = new Mechanism2d(2, 2);
	MechanismRoot2d armRoot = arm.getRoot("Arm Root", 1, 0.5);
	MechanismLigament2d armMech = armRoot.append(new MechanismLigament2d("Arm", 0.58, 90));
	MechanismLigament2d intakeMech = armMech.append(new MechanismLigament2d("Intake", 0.36, 26.0));
	MechanismLigament2d shooterMech = armMech.append(new MechanismLigament2d("Shooter", 0.11, 206));

	private double appliedVoltage = 0.0;

	@Override
	public void updateInputs(ArmIOInputs inputs) {
		armSim.update(0.02);
		inputs.velocityRadPerSec = getPositionRads();
		inputs.positionRads = armSim.getAngleRads();
		inputs.appliedVoltage = appliedVoltage;
		inputs.currentAmps = armSim.getCurrentDrawAmps();
	}

	// An initializing function
	@Override
	public void initial() {
		// Set up PID controllers
		armPIDController.disableContinuousInput();
		armPIDController.setTolerance(ArmConstants.kArmPIDTolerance);

		// Create Mechanism2D simulation stuff

		resetPIDControllers();
	}

	@Override
	public void update() {
		// Update Mechanism2D simulation stuff
        armMech.setAngle(getPositionRads());

		Logger.recordOutput("ArmMechanism", arm);
		SmartDashboard.putData("Mechanism", arm);
	}

	// Returns the current position of the arm in radians
	public double getPositionRads() {
		return armSim.getAngleRads();
	}

	// Calculates the voltage to run the motors at -- CHECK IF CALCULATE() CAN
	// RETURN VOLTS
	public double calculateInputVoltage(double setpoint) {
		return (armPIDController.calculate(getPositionRads(), setpoint)) / 100 * 12;
	}

	// Sets the input voltage for a motor
	@Override
	public void setInputVoltage(double volts) {
		appliedVoltage = volts;
		armSim.setInputVoltage(volts);
	}

	// Checks if the arm is past the from limit (could hit ground/front of robot)
	public boolean armPastFrontLimit() {
		return armSim.hasHitLowerLimit();
	}

	// Checks if the arm is past the back limit (could hit back/tip over robot)
	public boolean armPastBackLimit() {
		return armSim.hasHitUpperLimit();
	}

	// Check if the arm can be moved
	public boolean armWithinRange() {
		return !armPastFrontLimit() && !armPastBackLimit();
	}

	// Gets the preset angle in radians for the arm
	@Override
	public void getPreset() {
		// what function would i use in sim mode?
	}

	@Override
	public void stop() {
		appliedVoltage = 0.0;
		setInputVoltage(0.0);
	}

	@Override
	public void resetAbsoluteEncoders() {
		// don't need to do because simulation measures angle accurately
	}

	@Override
	public void resetPIDControllers() {
		armPIDController.reset(getPositionRads());
	}
}
