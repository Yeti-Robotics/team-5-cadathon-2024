package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClawSubsystem extends SubsystemBase {
    // Declare motor controllers
    private final TalonFX clawMotor;

    // Constructor
    public ClawSubsystem(int clawMotorID) {
        clawMotor = new TalonFX(clawMotorID);
        clawMotor.configFactoryDefault(); // Reset to default settings
        clawMotor.configPeakOutputForward(1.0); // Set max forward speed
        clawMotor.configPeakOutputReverse(-1.0); // Set max reverse speed
    }

    //Opens the claw by running the motor forward.
    public void openClaw() {
        clawMotor.set(ControlMode.PercentOutput, 0.5); // Adjust speed as needed
    }

    //Closes the claw by running the motor in reverse.
    public void closeClaw() {
        clawMotor.set(ControlMode.PercentOutput, -0.5); // Adjust speed as needed
    }

    //Stops the claw motor.
    public void stopClaw() {
        clawMotor.set(ControlMode.PercentOutput, 0.0);
    }

}
