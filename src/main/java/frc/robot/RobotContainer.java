package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;


public class RobotContainer {
    private final ClawSubsystem clawSubsystem = new ClawSubsystem(1);
    private final CommandSwerveDrivetrain swerveDrivetrain = null;
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    XboxController xboxController;

    public RobotContainer() {
        xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
        SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants(); // Replace with actual constants
        double odometryUpdateFrequency = 0.02; // Example value (50Hz)
        SwerveModuleConstants[] modules = new SwerveModuleConstants[4];
        modules[0] = new SwerveModuleConstants(); // Front-left module (IDs for drive, steer, encoder)
        modules[1] = new SwerveModuleConstants(); // Front-right module
        modules[2] = new SwerveModuleConstants(); // Back-left module
        modules[3] = new SwerveModuleConstants(); // Back-right module

        configureBindings();
        clawSubsystem.setDefaultCommand(
                new RunCommand(clawSubsystem::stopClaw, clawSubsystem)
        );

    }

    private void configureBindings() {
        // Open claw with Button A
        new JoystickButton(xboxController, XboxController.Button.kA.value)
                .onTrue(new InstantCommand(clawSubsystem::openClaw, clawSubsystem))
                .onFalse(new InstantCommand(clawSubsystem::stopClaw, clawSubsystem));

        // Close claw with Button B
        new JoystickButton(xboxController, XboxController.Button.kB.value)
                .onTrue(new InstantCommand(clawSubsystem::closeClaw, clawSubsystem))
                .onFalse(new InstantCommand(clawSubsystem::stopClaw, clawSubsystem));

        swerveDrivetrain.setDefaultCommand(new RunCommand(
                () -> swerveDrivetrain.drive(
                        -xboxController.getLeftY(), // Forward/backward translation
                        -xboxController.getLeftX(), // Side-to-side translation
                        xboxController.getRightX()  // Rotation
                ),
                swerveDrivetrain
        ));
        new JoystickButton(xboxController, XboxController.Button.kX.value)
                .onTrue(new InstantCommand(() -> swerveDrivetrain.drive(1.0, 0.0, 0.0), swerveDrivetrain))
                .onFalse(new InstantCommand(() -> swerveDrivetrain.drive(0.0, 0.0, 0.0), swerveDrivetrain));

        new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
                .onTrue(new InstantCommand(elevatorSubsystem::raiseStage1, elevatorSubsystem))
                .onFalse(new InstantCommand(elevatorSubsystem::stopStage1, elevatorSubsystem));

        new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
                .onTrue(new InstantCommand(elevatorSubsystem::lowerStage1, elevatorSubsystem))
                .onFalse(new InstantCommand(elevatorSubsystem::stopStage1, elevatorSubsystem));

        new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
                .onTrue(new InstantCommand(elevatorSubsystem::raiseStage2, elevatorSubsystem))
                .onFalse(new InstantCommand(elevatorSubsystem::stopStage2, elevatorSubsystem));

        new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
                .onTrue(new InstantCommand(elevatorSubsystem::lowerStage2, elevatorSubsystem))
                .onFalse(new InstantCommand(elevatorSubsystem::stopStage2, elevatorSubsystem));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}


