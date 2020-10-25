package frc.team832.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N1;

public class Constants {
    public static double kRobotMainLoopPeriod = 0.02;

    private Constants() {}

    public static class ShooterConstants{

        // Volts to break static friction
        public static final double kFlywheelKs = 0.0437;

        // Volts per (rotation per second)
        private static final double kFlywheelKvRotPerSec = 0.00217;

        // Volts per (radian per second)
        public static final double kFlywheelKv = 2 * Math.PI * (kFlywheelKvRotPerSec);

        // Volts per (rotation per second squared
        public static final double kFlywheelKaRotPerSecSq = 0.00103;

        // Volts per (radian per second squared)
        public static final double kFlywheelKa = 2 * Math.PI * (kFlywheelKaRotPerSecSq);

        public static final DCMotor kFlywheelGearbox = DCMotor.getNEO(2);

        // Gear ratio as output over input
        public static final double kFlywheelGearRatio = 26.0/50.0;

        // Moment of Inertia (kg meters squared)
        public static final double kFlywheelMoI = 0.00179;

        // Linear system based on characterization data
        public static final LinearSystem<N1, N1, N1> kFlywheelPlant_Char = LinearSystemId.identifyVelocitySystem(
                kFlywheelKv, kFlywheelKa);

        // Linear system based on MoI data (from CAD)
        public static final LinearSystem<N1, N1, N1> kFlywheelPlant_MoI = LinearSystemId.createFlywheelSystem(
                kFlywheelGearbox, kFlywheelMoI, kFlywheelGearRatio);

        public static double kFlywheelVelocityP = 0.0504;

        public static SimpleMotorFeedforward kFlywheelFF = new SimpleMotorFeedforward(
                kFlywheelKs, kFlywheelKvRotPerSec, kFlywheelKaRotPerSecSq);
    }
}
