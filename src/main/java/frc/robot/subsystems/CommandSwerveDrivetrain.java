package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandSwerveDrivetrain<RobotDataPublisher> extends SwerveDrivetrain implements Subsystem {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private double lastSimTime;
    private final boolean hasAppliedPerspective = false;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    public static final double SUPPLY_CURRENT_LIMIT_CURRENT_THRESHOLD = 65;
    public static final double SUPPLY_CURRENT_LIMIT_TIME_THRESHOLD = 0.1;

    public static final double PEAK_FORWARD_VOLTAGE = 12.0;
    public static final double PEAK_REVERSE_VOLTAGE = -12.0;

    public static final double SWERVE_X_REDUCTION = 1.0 / 6.75;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); //0.1016

    public static final double MaFxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * SWERVE_X_REDUCTION * WHEEL_DIAMETER * Math.PI; //placeholder

    private static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.25); //PLACEHOLDER
    private static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.25); //PLACEHOLDER

    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
            new SwerveDriveKinematics(
                    // Front left
                    new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0,
                            DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                    // Front right
                    new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0,
                            -DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                    // Back left
                    new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0,
                            DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
                    // Back right
                    new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0,
                            -DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
            );


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        setDriveCurrentLimits();
        setDriveVoltageLimits();
        setAzimuthCurrentLimits();
        setAzimuthVoltageLimits();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    public void setDriveCurrentLimits() {
        var currentLimitConfigs = new CurrentLimitsConfigs();

        for (var module : Modules) {
            var currentConfig = module.getDriveMotor().getConfigurator();
            currentConfig.refresh(currentLimitConfigs);

            currentLimitConfigs.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
            currentLimitConfigs.SupplyCurrentLimitEnable = SUPPLY_CURRENT_LIMIT_ENABLE;
            currentLimitConfigs.SupplyCurrentThreshold = SUPPLY_CURRENT_LIMIT_CURRENT_THRESHOLD;
            currentLimitConfigs.SupplyTimeThreshold = SUPPLY_CURRENT_LIMIT_TIME_THRESHOLD;

            currentConfig.apply(currentLimitConfigs);
        }
    }

    public void setAzimuthCurrentLimits() {
        var currentLimitConfigs = new CurrentLimitsConfigs();

        for (var module : Modules) {
            var currentConfig = module.getSteerMotor().getConfigurator();
            currentConfig.refresh(currentLimitConfigs);

            currentLimitConfigs.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
            currentLimitConfigs.SupplyCurrentLimitEnable = SUPPLY_CURRENT_LIMIT_ENABLE;
            currentLimitConfigs.SupplyCurrentThreshold = SUPPLY_CURRENT_LIMIT_CURRENT_THRESHOLD;
            currentLimitConfigs.SupplyTimeThreshold = SUPPLY_CURRENT_LIMIT_TIME_THRESHOLD;

            currentConfig.apply(currentLimitConfigs);
        }
    }

    public void setDriveVoltageLimits() {
        var voltageLimitConfigs = new VoltageConfigs();

        for (var module : Modules) {
            var currentConfig = module.getDriveMotor().getConfigurator();

            currentConfig.refresh(voltageLimitConfigs);

            voltageLimitConfigs.PeakForwardVoltage = PEAK_FORWARD_VOLTAGE;
            voltageLimitConfigs.PeakReverseVoltage = PEAK_REVERSE_VOLTAGE;

            currentConfig.apply(voltageLimitConfigs);
        }
    }

    public void setAzimuthVoltageLimits() {
        var voltageLimitConfigs = new VoltageConfigs();

        for (var module : Modules) {
            var currentConfig = module.getSteerMotor().getConfigurator();

            currentConfig.refresh(voltageLimitConfigs);

            voltageLimitConfigs.PeakForwardVoltage = PEAK_FORWARD_VOLTAGE;
            voltageLimitConfigs.PeakReverseVoltage = PEAK_REVERSE_VOLTAGE;

            currentConfig.apply(voltageLimitConfigs);
        }
    }

    public void run(double v, double v1, double rightX) {
    }

    public void drive(double v, double v1, double v2) {
    }
}