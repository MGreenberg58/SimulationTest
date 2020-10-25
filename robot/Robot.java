/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team832.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.team832.robot.subsystems.Shooter;

public class Robot extends TimedRobot {

  public static RobotContainer robotContainer = new RobotContainer();

  @Override
  public void robotInit() {
    robotContainer.shooter.encoder.setDistancePerPulse(
            2.0 * Math.PI / 4096.0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationPeriodic(){
    double shooterCurrent = robotContainer.shooter.getDrawnCurrentAmps();
    double loadedVoltage = BatterySim.calculateLoadedBatteryVoltage(12.8, 0.02, shooterCurrent);
    RoboRioSim.setVInVoltage(loadedVoltage);
    SmartDashboard.putNumber("robot/current_draw", shooterCurrent);
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    robotContainer.shooter.resetLoop();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit(){
    CommandScheduler.getInstance().cancelAll();
  }


}
