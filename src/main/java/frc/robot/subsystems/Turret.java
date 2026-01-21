package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.telemetry.SmartMotorControllerTelemetryConfig;
import yams.units.CRTAbsoluteEncoder;
import yams.units.CRTAbsoluteEncoderConfig;

/**
 * Turret that uses YAMS CRT
 */
public class Turret extends SubsystemBase{
  /** Manually rerun CRT seeding. */
  private static final String RERUN_SEED = "Turret/CRT/RerunSeed";

  private final TalonFX turretMotor;
  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig;
  private final SmartMotorControllerConfig motorConfig;
  private final SmartMotorController motor;
  private final MechanismPositionConfig robotToMechanism;
  private final PivotConfig pivotConfig;
  private final Pivot turret;
  private final CANcoder cancoderA;
  private final CANcoder cancoderB;
  private final StatusSignal<Angle> absPosition1Signal;
  private final StatusSignal<Angle> absPosition2Signal;
  private final CRTAbsoluteEncoderConfig crtConfig;

  private boolean rotorSeededFromAbs = false;
  private double lastSeededTurretDeg = Double.NaN;
  private double lastSeedError = Double.NaN;
  private double lastAbsA = Double.NaN;
  private double lastAbsB = Double.NaN;
  private String lastSeedStatus = "NOT_ATTEMPTED";

  public Turret() {
    turretMotor = new TalonFX(1, "rio");

    cancoderA = new CANcoder(2, "rio");
    cancoderB = new CANcoder(3, "rio");

    var cancoderConfigurationA = new CANcoderConfiguration();
    cancoderConfigurationA.MagnetSensor.MagnetOffset = -0.01513671875;
    cancoderA.getConfigurator().apply(cancoderConfigurationA);

    var cancoderConfigurationB = new CANcoderConfiguration();
    cancoderConfigurationB.MagnetSensor.MagnetOffset = -0.112548828125;
    cancoderB.getConfigurator().apply(cancoderConfigurationB);

    absPosition1Signal = cancoderA.getAbsolutePosition();
    absPosition2Signal = cancoderB.getAbsolutePosition();

    motorTelemetryConfig =
        new SmartMotorControllerTelemetryConfig()
            .withMechanismPosition()
            .withRotorPosition()
            .withRotorVelocity()
            .withMechanismLowerLimit()
            .withMechanismUpperLimit();

    motorConfig =
        new SmartMotorControllerConfig(this)
            .withClosedLoopController(
                80, 0, 0, DegreesPerSecond.of(45), DegreesPerSecondPerSecond.of(45))
            .withSimClosedLoopController(
                130, 0, 3.4, DegreesPerSecond.of(1000), DegreesPerSecondPerSecond.of(1500))
            .withSoftLimit(Degrees.of(0), Degrees.of(700))
            .withFeedforward(new SimpleMotorFeedforward(0.15,1.2))
            .withGearing(
                new MechanismGearing(
                    37.5))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("TurretMotorV2", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withSupplyCurrentLimit(Amps.of(4))
            .withMotorInverted(false)
            .withControlMode(ControlMode.CLOSED_LOOP);

    motor = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX44(1), motorConfig);

    robotToMechanism =
        new MechanismPositionConfig()
            .withMaxRobotHeight(Meters.of(1.5))
            .withMaxRobotLength(Meters.of(0.75))
            .withRelativePosition(
                new Translation3d(
                    Meters.of(-0.1524), // back from robot center
                    Meters.of(0.0), // centered left/right
                    Meters.of(0.451739) // up from the floor reference
                    ));

    pivotConfig =
        new PivotConfig(motor)
            .withHardLimit(Degrees.of(0), Degrees.of(700))
            .withTelemetry("Turret", TelemetryVerbosity.HIGH)
            .withStartingPosition(Degrees.of(0))
            .withMechanismPositionConfig(robotToMechanism)
            .withMOI(0.2);

    turret = new Pivot(pivotConfig);
    crtConfig = buildCrtConfig();
    logCrtConfigTelemetry();
    SmartDashboard.putBoolean(RERUN_SEED, false);
  }


  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  public double getRobotRelativeYawRadians() {
    return getAngle().in(edu.wpi.first.units.Units.Radians);
  }

  /**
   * Forces a CRT reseed attempt
   */
  public void rerunCrtSeed() {
    rotorSeededFromAbs = false;
    SmartDashboard.putNumber("Turret/CRT/ManualRerunTimestampSec", Timer.getFPGATimestamp());
    attemptRotorSeedFromCANCoders();
  }

  public void periodic() {
    if (SmartDashboard.getBoolean(RERUN_SEED, false)) {
      SmartDashboard.putBoolean(RERUN_SEED, false);
      rerunCrtSeed();
    }
    SmartDashboard.putNumber(
        "Turret/CRT/CurrentPositionDeg", motor.getMechanismPosition().in(Degrees));
    if (!rotorSeededFromAbs) {
      attemptRotorSeedFromCANCoders();
    }
    turret.updateTelemetry();
  }

  public void simulationPeriodic() {
    turret.simIterate();
  }

  /**
   * Tries to solve turret position via CRT and seed the relative encoder with the result.
   * Reads both CANCoder values, runs the solver, updates the SmartMotorController encoder,
   * and publishes CRT status to the dashboard.
   */
  private void attemptRotorSeedFromCANCoders() {
    AbsSensorRead absRead = readAbsSensors();
    if (!absRead.ok()) {
      if (!"NO_DEVICES".equals(absRead.status())) {
        SmartDashboard.putString("Turret/CRT/SeedStatus", absRead.status());
      }
      lastSeedStatus = absRead.status();
      return;
    }

    double absA = absRead.absA();
    double absB = absRead.absB();
    lastAbsA = absA;
    lastAbsB = absB;

    var solver = new CRTAbsoluteEncoder(crtConfig);
    var solvedAngle = solver.getAngleOptional();

    SmartDashboard.putNumber("Turret/CRT/AbsA", absA);
    SmartDashboard.putNumber("Turret/CRT/AbsB", absB);
    SmartDashboard.putString("Turret/CRT/SolverStatus", solver.getLastStatus());
    SmartDashboard.putNumber("Turret/CRT/SolverErrorRot", solver.getLastErrorRotations());
    SmartDashboard.putNumber("Turret/CRT/SolverIterations", solver.getLastIterations());

    if (solvedAngle.isEmpty()) {
      SmartDashboard.putBoolean("Turret/CRT/SolutionFound", false);
      lastSeedStatus = solver.getLastStatus();
      return;
    }

    Angle solvedAngleValue = solvedAngle.get();
    motor.setEncoderPosition(solvedAngleValue);
    rotorSeededFromAbs = true;
    lastSeededTurretDeg = solvedAngleValue.in(Degrees);
    lastSeedError = solver.getLastErrorRotations();
    double currentTalonDeg = motor.getMechanismPosition().in(Degrees);
    double talonDeltaDeg = currentTalonDeg - lastSeededTurretDeg;
    double encoder2Ratio = crtConfig.getEncoder2RotationsPerMechanismRotation();
    double expectedMechanismErrDeg =
        Double.isFinite(encoder2Ratio) && Math.abs(encoder2Ratio) > 1e-12
            ? (lastSeedError / encoder2Ratio) * 360.0
            : Double.NaN;
    SmartDashboard.putBoolean("Turret/CRT/SolutionFound", true);
    SmartDashboard.putNumber("Turret/CRT/SeededTurretDeg", lastSeededTurretDeg);
    SmartDashboard.putNumber("Turret/CRT/MatchErrorRot", lastSeedError);
    SmartDashboard.putNumber("Turret/CRT/TalonAfterSeedDeg", currentTalonDeg);
    SmartDashboard.putNumber("Turret/CRT/TalonMinusSeedDeg", talonDeltaDeg);
    SmartDashboard.putNumber("Turret/CRT/ExpectedDeltaDegFromMatchError", expectedMechanismErrDeg);

    lastSeedStatus = "OK";
    SmartDashboard.putString("Turret/CRT/SeedStatus", lastSeedStatus);
    SmartDashboard.putBoolean("Turret/CRT/Seeded", rotorSeededFromAbs);
  }

  /**
   * Reads both absolute encoders and returns their rotations plus a status.
   */
  private AbsSensorRead readAbsSensors() {
    boolean haveDevices = cancoderA != null && cancoderB != null;
    if (haveDevices) {
      var status = BaseStatusSignal.refreshAll(absPosition1Signal, absPosition2Signal);
      if (status.isOK()) {
        return new AbsSensorRead(
            true,
            absPosition1Signal.getValue().in(Rotations),
            absPosition2Signal.getValue().in(Rotations),
            status.toString());
      }
      return new AbsSensorRead(false, Double.NaN, Double.NaN, status.toString());
    }

    return new AbsSensorRead(false, Double.NaN, Double.NaN, "NO_DEVICES");
  }

  /**
   * Build the CRT config
   */
  private CRTAbsoluteEncoderConfig buildCrtConfig() {
    return new CRTAbsoluteEncoderConfig(
            absPosition1Signal::getValue,
            absPosition2Signal::getValue)
        .withCommonDriveGear(
          9, 
          50, 
          33, 
          34)
        .withMechanismRange(Rotations.of(0.0), Rotations.of(2))
        .withMatchTolerance(Rotations.of(0.005))
        .withCrtGearRecommendationConstraints(
          1.2, 
          15, 
          60, 
          40);
  }

  /**
   * Publish CRT config-derived values for debugging coverage/ratios.
   */
  private void logCrtConfigTelemetry() {
    double mechanismRangeRot =
        crtConfig.getMaxMechanismRotations() - crtConfig.getMinMechanismRotations();
    SmartDashboard.putNumber(
        "Turret/CRT/Config/RatioA", crtConfig.getEncoder1RotationsPerMechanismRotation());
    SmartDashboard.putNumber(
        "Turret/CRT/Config/RatioB", crtConfig.getEncoder2RotationsPerMechanismRotation());
    SmartDashboard.putNumber(
        "Turret/CRT/Config/UniqueCoverageRot",
        crtConfig.getUniqueCoverageRotations().orElse(Double.NaN));
    SmartDashboard.putBoolean(
        "Turret/CRT/Config/CoverageSatisfiesRange",
        crtConfig
            .getUniqueCoverageRotations()
            .map(coverage -> coverage >= mechanismRangeRot)
            .orElse(false));
    SmartDashboard.putNumber("Turret/CRT/Config/RequiredRangeRot", mechanismRangeRot);

    var configPair = crtConfig.getRecommendedCrtGearPair();
    SmartDashboard.putBoolean("Turret/CRT/Config/RecommendedPairFound", configPair.isPresent());
    if (configPair.isPresent()) {
      var pair = configPair.get();
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearA", pair.gearA());
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearB", pair.gearB());
      SmartDashboard.putNumber(
          "Turret/CRT/Config/Reccomender/RecommendedCoverageRot", pair.coverageRot());
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedLcm", pair.lcm());
      SmartDashboard.putBoolean(
          "Turret/CRT/Config/Reccomender/RecommendedCoprime",
          CRTAbsoluteEncoderConfig.isCoprime(pair.gearA(), pair.gearB()));
      SmartDashboard.putNumber(
          "Turret/CRT/Config/Reccomender/RecommendedIterations", pair.theoreticalIterations());
    }
  }

  private static record AbsSensorRead(boolean ok, double absA, double absB, String status) {}
}
