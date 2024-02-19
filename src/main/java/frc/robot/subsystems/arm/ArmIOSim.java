package frc.robot.subsystems.arm;

import static frc.robot.Constants.GainsConstants.armGains;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {
	private final SingleJointedArmSim armSim;
	private final ProfiledPIDController armPIDController;
	private ArmFeedforward armFF;

	private final Mechanism2d arm;
	private final MechanismRoot2d armRoot;
	private final MechanismLigament2d superstructureMech, armMech;

	private double appliedVoltage = 0.0;

	public ArmIOSim() {
		System.out.println("[Init] Creating ArmIOSim");

		armSim = new SingleJointedArmSim(DCMotor.getNEO(2), ArmConstants.kArmReduction,
				ArmConstants.kArmJKgMetersSquared, ArmConstants.kArmLength, ArmConstants.kMinArmAngle,
				ArmConstants.kMaxArmAngle, true, Math.PI / 2);

		armPIDController = new ProfiledPIDController(armGains.kP(), armGains.kI(), armGains.kD(),
				new Constraints(ArmConstants.kMaxSpeed.get(), ArmConstants.kMaxAcceleration.get()));
		armPIDController.disableContinuousInput();
		armPIDController.setTolerance(ArmConstants.kArmPIDTolerance.get());

		armFF = new ArmFeedforward(armGains.ffkS(), armGains.ffkG(), armGains.ffkV(), armGains.ffkA());

		// Mechanism visualizer
		arm = new Mechanism2d(2, 2);
		armRoot = arm.getRoot("Arm Root", 1, 0.5);
		superstructureMech = armRoot.append(new MechanismLigament2d("Superstructure", 0.2, 90));
		armMech = superstructureMech.append(new MechanismLigament2d("Arm", 0.58, 90));
		armMech.append(new MechanismLigament2d("Intake", 0.36, 64.0));
		armMech.append(new MechanismLigament2d("Shooter", 0.11, 244.0));

		resetPIDControllers();
	}

	@Override
	public void updateInputs(ArmIOInputs inputs) {
		armSim.update(0.02);

		inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
		inputs.positionRads = getPositionRads();
		inputs.appliedVoltage = appliedVoltage;
		inputs.currentAmps = new double[]{armSim.getCurrentDrawAmps()};
		inputs.withinRange = armWithinRange();

		// Update mechanism visualizer
		armMech.setAngle(getPositionRads());
		Logger.recordOutput("Arm/Mechanism", arm);
	}

	@Override
	public void setInputVoltage(double volts) {
		appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
		armSim.setInputVoltage(appliedVoltage);
	}

	@Override
	public void stop() {
		setInputVoltage(0.0);
	}

	@Override
	public void setGains(double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {
		armPIDController.setPID(kP, kI, kD);
		armFF = new ArmFeedforward(ffkS, ffkG, ffkV, ffkA);
	}

	@Override
	public void update() {
	}

	@Override
	public double getPositionRads() {
		return armSim.getAngleRads();
	}

	@Override
	public void setPosition(double setpoint) {
		double volts = (armPIDController.calculate(getPositionRads(), setpoint)
				+ armFF.calculate(getPositionRads(), setpoint));
		setInputVoltage(volts);
	}

	@Override
	public boolean armPastFrontLimit() {
		return armSim.hasHitLowerLimit();
	}

	@Override
	public boolean armPastBackLimit() {
		return armSim.hasHitUpperLimit();
	}

	@Override
	public void resetPIDControllers() {
		armPIDController.reset(getPositionRads());
	}
}
