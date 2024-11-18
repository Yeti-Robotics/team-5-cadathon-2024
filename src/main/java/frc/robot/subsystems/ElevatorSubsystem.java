package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    // Motor controllers for the stages
    private final TalonFX stage1Motor;
    private final TalonFX stage2Motor;

    // Limit switches for each stage
    private final DigitalInput stage1UpperLimit;
    private final DigitalInput stage1LowerLimit;
    private final DigitalInput stage2UpperLimit;
    private final DigitalInput stage2LowerLimit;

    // Constructor
    public ElevatorSubsystem() {
        // Initialize motor controllers and limit switches here
        stage1Motor = new TalonFX(0); // Replace with actual port numbers
        stage2Motor = new TalonFX(1); // Replace with actual port numbers

        // Initialize limit switches (replace with actual ports)
        stage1UpperLimit = new DigitalInput(0);
        stage1LowerLimit = new DigitalInput(1);
        stage2UpperLimit = new DigitalInput(2);
        stage2LowerLimit = new DigitalInput(3);
    }

    // Methods for controlling the elevator
    public void raiseStage1() {
        // Check if the upper limit switch is hit for stage 1
        if (!stage1UpperLimit.get()) {
            stage1Motor.set(0.5); // Set motor speed to raise
        } else {
            stage1Motor.set(0); // Stop the motor if the upper limit is hit
        }
    }

    public void lowerStage1() {
        // Check if the lower limit switch is hit for stage 1
        if (!stage1LowerLimit.get()) {
            stage1Motor.set(-0.5); // Set motor speed to lower
        } else {
            stage1Motor.set(0); // Stop the motor if the lower limit is hit
        }
    }

    // Raise the second stage if upper limit is not reached
    public void raiseStage2() {
        if (!stage2UpperLimit.get()) {
            stage2Motor.set(0.5); // Set motor speed to raise
        } else {
            stage2Motor.set(0); // Stop motor if upper limit switch is hit
        }
    }

    // Lower the second stage if lower limit is not reached
    public void lowerStage2() {
        if (!stage2LowerLimit.get()) {
            stage2Motor.set(-0.5); // Set motor speed to lower
        } else {
            stage2Motor.set(0); // Stop motor if lower limit switch is hit
        }
    }

    // Stop the first stage motor
    public void stopStage1() {
        stage1Motor.set(0); // Stop the motor
    }

    // Stop the second stage motor
    public void stopStage2() {
        stage2Motor.set(0); // Stop the motor
    }

    @Override
    public void periodic() {
        // This method will be called periodically by the scheduler
        // You can use this to update system states, log info, etc.
    }
}
