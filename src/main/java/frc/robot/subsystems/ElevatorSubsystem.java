package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    // Motor controllers for the stages
    private TalonFX stage1Motor;
    private TalonFX stage2Motor;

    // Limit switches for each stage
    private DigitalInput stage1UpperLimit;
    private DigitalInput stage1LowerLimit;
    private DigitalInput stage2UpperLimit;
    private DigitalInput stage2LowerLimit;

    // Constructor
    public void Elevator() {
        // Initialize motor controllers and limit switches here
        stage1Motor = new TalonFX(0); // Replace with actual port numbers
        stage2Motor = new TalonFX(1);
        // ...
    }

    // Methods for controlling the elevator
    public void raiseStage1() {
        // Check if the upper limit switch is hit
        if (!stage1UpperLimit.get()) {
            stage1Motor.set(0.5); // Set motor speed to raise
        } else {
            stage1Motor.set(0); // Stop the motor
        }
    }

    public void lowerStage1() {
        // Check if the lower limit switch is hit
        if (!stage1LowerLimit.get()) {
            stage1Motor.set(-0.5); // Set motor speed to lower
        } else {
            stage1Motor.set(0); // Stop the motor
        }
    }

    // ... similar methods for stage 2

    @Override
    public void periodic() {
        // This method will be called periodically by the scheduler
    }
}