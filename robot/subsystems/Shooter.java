package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.team832.robot.Constants;

public class Shooter extends SubsystemBase {
    private static final double Kv = 0.1;
    private static final double Ka = 0.01;

    private final LinearSystem<N1, N1, N1> m_flywheelPlant = LinearSystemId.identifyVelocitySystem(Kv, Ka);


    public final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
            Nat.N1(), Nat.N1(),
            m_flywheelPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is
            VecBuilder.fill(0.01), // How accurate we think our encoder data is
            0.020);

    public final LinearQuadraticRegulator<N1, N1, N1> m_controller
            = new LinearQuadraticRegulator<>(m_flywheelPlant,
            VecBuilder.fill(8.0), // Velocity error tolerance
            VecBuilder.fill(12.0), // Control effort (voltage) tolerance
            0.020);

    public final LinearSystemLoop<N1, N1, N1> loop = new LinearSystemLoop<>(
            m_flywheelPlant,
            m_controller,
            m_observer,
            12.0,
            0.020);


    public final Encoder encoder = new Encoder(2, 3);

    public final SpeedController motor = new PWMVictorSPX(0);

    private final PIDController flywheelPIDController;

    private FlywheelSim flywheelSim;

    private EncoderSim encoderSim;

    public Shooter() {
        encoder.reset();

        flywheelPIDController = new PIDController(Constants.ShooterConstants.kFlywheelVelocityP, 0, 0, Constants.kRobotMainLoopPeriod);
        flywheelPIDController.setTolerance(Constants.ShooterConstants.kFlywheelVelocityP, 50);

        SmartDashboard.putData("flywheel/pid", flywheelPIDController);

        if (RobotBase.isSimulation()) {
            flywheelSim = new FlywheelSim(
                    Constants.ShooterConstants.kFlywheelPlant_Char,
                    Constants.ShooterConstants.kFlywheelGearbox,
                    Constants.ShooterConstants.kFlywheelGearRatio,
                    null
            );

            encoderSim = new EncoderSim(encoder);
        }

    }

    public void resetLoop(){
       loop.reset(VecBuilder.fill(encoder.getRate()));
    }


    public void periodic() {
        if (RobotBase.isSimulation()) {
            flywheelSim.setInput(motor.get() * RobotController.getBatteryVoltage());
            flywheelSim.update(Constants.kRobotMainLoopPeriod);
            encoderSim.setRate(flywheelSim.getAngularVelocityRadPerSec());
        }

        double flywheelFFEffort = Constants.ShooterConstants.kFlywheelFF.calculate(flywheelPIDController.getSetpoint()) * Constants.ShooterConstants.kFlywheelGearRatio;

        double flywheelPIDEffort = MathUtil.clamp(flywheelPIDController.calculate(flywheelSim.getAngularVelocityRPM()), -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());

        double flywheelEffort = flywheelFFEffort + flywheelPIDEffort;

        SmartDashboard.putNumber("flywheel/pid_error", flywheelPIDController.getPositionError());
        SmartDashboard.putNumber("flywheel/ff_effort", flywheelFFEffort);
        SmartDashboard.putNumber("flywheel/pid_effort", flywheelPIDEffort);
        SmartDashboard.putNumber("flywheel/motor_get", motor.get());
        SmartDashboard.putBoolean("flywheel/at_setpoint", flywheelPIDController.atSetpoint());
        SmartDashboard.putNumber("flywheel/total_effort", flywheelEffort);
        SmartDashboard.putNumber("flywheel/flywheel_RPM", Units.radiansPerSecondToRotationsPerMinute(encoder.getRate()));

        motor.set(flywheelEffort / RobotController.getBatteryVoltage());
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return flywheelSim.getCurrentDrawAmps();
        } else return 0;
    }

    public void setFlywheelRPMSetpoint(double rpm) {
        flywheelPIDController.setSetpoint(rpm);
    }
}
