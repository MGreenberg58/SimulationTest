package frc.team832.robot;

import frc.team832.lib.driverinput.controllers.Xbox360Controller;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.TemplateSubsystem;

public class RobotContainer {

    public final TemplateSubsystem templateSubsystem= new TemplateSubsystem();
    public final Shooter shooter = new Shooter();

    public static final Xbox360Controller controller = new Xbox360Controller(0);

    public RobotContainer() {

        if (templateSubsystem.initializedSuccessfully) {
            System.out.println("Template Subsys - INIT OK");
        } else {
            System.out.println("Template Subsys - INIT FAIL");
        }

        assignButtons();
    }

    public void assignButtons(){
        controller.bButton.whenPressed(() -> shooter.setFlywheelRPMSetpoint(4000), shooter).whenReleased(() -> shooter.setFlywheelRPMSetpoint(0), shooter);
    }
}