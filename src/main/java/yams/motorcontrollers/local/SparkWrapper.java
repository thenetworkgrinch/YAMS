package yams.motorcontrollers.local;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.Optional;
import java.util.function.Supplier;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public class SparkWrapper extends SmartMotorController
{

  /**
   * Spark configuration retry count.
   */
  private final int                               CONFIG_RETRIES          = 4;
  /**
   * Spark configuration retry delay.
   */
  private final double                            CONFIG_RETRY_DELAY      = Milliseconds.of(5).in(Seconds);
  /**
   * Spark motor controller
   */
  private final SparkBase                         spark;
  /**
   * Motor type.
   */
  private final DCMotor            motor;
  /**
   * Spark base configuration.
   */
  private final SparkBaseConfig    sparkBaseConfig;
  /**
   * Spark relative encoder.
   */
  private final RelativeEncoder    sparkRelativeEncoder;
  /**
   * Spark relative encoder sim object.
   */
  private       Optional<SparkRelativeEncoderSim> sparkRelativeEncoderSim = Optional.empty();
  /**
   * Spark simulation.
   */
  private       Optional<SparkSim> sparkSim = Optional.empty();
  /**
   * Spark absolute encoder.
   */
  private       Optional<AbsoluteEncoder>         sparkAbsoluteEncoder    = Optional.empty();
  /**
   * Spark absolute encoder sim object
   */
  private       Optional<SparkAbsoluteEncoderSim> sparkAbsoluteEncoderSim = Optional.empty();

  public enum SparkBaseType
  {
    SPARK_MAX, SPARK_FLEX
  }

  public SparkWrapper(int id, SparkBaseType type)
  {
    if (type == SparkBaseType.SPARK_MAX)
    {
      spark = new SparkMax(id, MotorType.kBrushless);
      sparkBaseConfig = new SparkMaxConfig();
    } else if (type == SparkBaseType.SPARK_FLEX)
    {
      spark = new SparkFlex(id, MotorType.kBrushless);
      sparkBaseConfig = new SparkFlexConfig();
    } else
    {
      throw new IllegalArgumentException("[ERROR] Unsupported controller type: " + type.name());
    }
    motor = DCMotor.getNEO(1); // Default to NEO motor type
    sparkRelativeEncoder = spark.getEncoder();
  }

  /**
   * Create a {@link SmartMotorController} from {@link SparkMax} or {@link SparkFlex}
   *
   * @param controller {@link SparkMax} or {@link SparkFlex}
   * @param motor      {@link DCMotor} controller by the {@link SparkFlex} or {@link SparkMax}. Must be a brushless
   *                   motor.
   * @param config     {@link SmartMotorControllerConfig} to apply.
   */
  public SparkWrapper(SparkBase controller, DCMotor motor, SmartMotorControllerConfig config)
  {
    if (controller instanceof SparkMax)
    {
      sparkBaseConfig = new SparkMaxConfig();
    } else if (controller instanceof SparkFlex)
    {
      sparkBaseConfig = new SparkFlexConfig();
    } else
    {
      throw new IllegalArgumentException(
          "[ERROR] Unsupported controller type: " + controller.getClass().getSimpleName());
    }

    this.motor = motor;
    spark = controller;
    sparkRelativeEncoder = controller.getEncoder();
    setupSimulation();
    applyConfig(config);
    checkConfigSafety();

  }

  /**
   * Run the configuration until it succeeds or times out.
   *
   * @param config Lambda supplier returning the error state.
   * @return Successful configuration
   */
  private boolean configureSpark(Supplier<REVLibError> config)
  {
    for (int i = 0; i < CONFIG_RETRIES; i++)
    {
      if (config.get() == REVLibError.kOk)
      {
        return true;
      }
      Timer.delay(CONFIG_RETRY_DELAY);
    }
    return false;
  }

  @Override
  public void setupSimulation()
  {
    if (RobotBase.isSimulation())
    {
      sparkSim = Optional.of(new SparkSim(spark, motor));
      sparkRelativeEncoderSim = Optional.of(sparkSim.get().getRelativeEncoderSim());
    }
  }


  @Override
  public void seedRelativeEncoder()
  {
    if (sparkAbsoluteEncoder.isPresent())
    {
      sparkRelativeEncoder.setPosition(sparkAbsoluteEncoder.get().getPosition());
      sparkRelativeEncoderSim.ifPresent(sparkRelativeEncoderSim -> sparkRelativeEncoderSim.setPosition(
          sparkAbsoluteEncoder.get().getPosition()));
    }
  }

  @Override
  public void synchronizeRelativeEncoder()
  {
    if (config.getFeedbackSynchronizationThreshold().isPresent())
    {
      if (sparkAbsoluteEncoder.isPresent())
      {
        if (!Rotations.of(sparkRelativeEncoder.getPosition()).isNear(Rotations.of(sparkAbsoluteEncoder.get()
                                                                                                      .getPosition()),
                                                                     config.getFeedbackSynchronizationThreshold()
                                                                           .get()))
        {
          seedRelativeEncoder();
        }
      }
    }
  }

  @Override
  public void simIterate(AngularVelocity mechanismVelocity)
  {
    if (RobotBase.isSimulation())
    {
      sparkSim.ifPresent(sim -> sim.iterate(mechanismVelocity.in(RotationsPerSecond),
                                            RoboRioSim.getVInVoltage(),
                                            config.getClosedLoopControlPeriod().in(Second)));
      sparkRelativeEncoderSim.ifPresent(sim -> sim.iterate(mechanismVelocity.in(RotationsPerSecond),
                                                           config.getClosedLoopControlPeriod().in(Seconds)));
      sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.iterate(mechanismVelocity.in(
          RotationsPerSecond), config.getClosedLoopControlPeriod().in(Seconds)));
    }
  }

  @Override
  public void setEncoderVelocity(LinearVelocity velocity)
  {
    setEncoderVelocity(config.convertToMechanism(velocity));
  }

  @Override
  public void setEncoderPosition(Angle angle)
  {
    if (sparkAbsoluteEncoder.isPresent())
    {
      sparkBaseConfig.absoluteEncoder.zeroOffset(getMechanismPosition().minus(angle).in(Rotations));
      sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.setPosition(angle.in(Rotations)));
    }
    sparkRelativeEncoder.setPosition(angle.in(Rotations));
    sparkRelativeEncoderSim.ifPresent(relativeEncoderSim -> relativeEncoderSim.setPosition(angle.in(Rotations)));
  }

  @Override
  public void setEncoderVelocity(AngularVelocity velocity)
  {
    sparkRelativeEncoderSim.ifPresent(relativeEncoderSim -> relativeEncoderSim.setVelocity(velocity.in(
        RotationsPerSecond)));
    sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.setVelocity(velocity.in(
        RotationsPerSecond)));
  }

  @Override
  public void setEncoderPosition(Distance distance)
  {
    setEncoderPosition(config.convertToMechanism(distance));
  }

  @Override
  public void setPosition(Angle angle)
  {
    setpointPosition = angle == null ? Optional.empty() : Optional.of(angle);
  }

  @Override
  public void setPosition(Distance distance)
  {
    setPosition(config.convertToMechanism(distance));
  }

  @Override
  public void setVelocity(LinearVelocity velocity)
  {
    setVelocity(config.convertToMechanism(velocity));
  }

  @Override
  public void setVelocity(AngularVelocity angle)
  {
    setpointVelocity = angle == null ? Optional.empty() : Optional.of(angle);
  }

  @Override
  public boolean applyConfig(SmartMotorControllerConfig config)
  {

    this.config = config;
    pidController = config.getClosedLoopController();

    // Handle simple pid vs profile pid controller.
    if (pidController.isEmpty())
    {
      simplePidController = config.getSimpleClosedLoopController();
      if (simplePidController.isEmpty())
      {
        throw new IllegalArgumentException("[ERROR] closed loop controller must not be empty");
      }
    }

    // Handle closed loop controller thread
    if (closedLoopControllerThread == null)
    {
      closedLoopControllerThread = new Notifier(this::iterateClosedLoopController);

    } else
    {
      closedLoopControllerThread.stop();
    }
    if (config.getTelemetryName().isPresent())
    {
      closedLoopControllerThread.setName(config.getTelemetryName().get());
    }
    if (config.getMotorControllerMode() == ControlMode.CLOSED_LOOP)
    {
      closedLoopControllerThread.startPeriodic(config.getClosedLoopControlPeriod().in(Second));
    } else
    {
      closedLoopControllerThread.stop();
    }
    // Calculate Spark conversion factors
    double positionConversionFactor = config.getGearing().getRotorToMechanismRatio();
    double velocityConversionFactor = config.getGearing().getRotorToMechanismRatio() / 60.0;

    // Set base config options
    sparkBaseConfig.openLoopRampRate(config.getOpenLoopRampRate().in(Seconds));
    sparkBaseConfig.closedLoopRampRate(config.getClosedLoopRampRate().in(Seconds));
    sparkBaseConfig.inverted(config.getMotorInverted());
    sparkBaseConfig.encoder.positionConversionFactor(positionConversionFactor);
    sparkBaseConfig.encoder.velocityConversionFactor(velocityConversionFactor);
    if (config.getEncoderInverted())
    {
      throw new IllegalArgumentException("[ERROR] Spark relative encoder cannot be inverted!");
    }
    // Throw warning about supply stator limits on Spark's
    if (config.getSupplyStallCurrentLimit().isPresent())
    {
      DriverStation.reportError("[WARNING] Supply stall currently not supported on Spark", true);
    }
    // Handle stator current limit.
    if (config.getStatorStallCurrentLimit().isPresent())
    {
      sparkBaseConfig.smartCurrentLimit(config.getStatorStallCurrentLimit().getAsInt());
    }
    // Handle voltage compensation.
    if (config.getVoltageCompensation().isPresent())
    {
      sparkBaseConfig.voltageCompensation(config.getVoltageCompensation().get().in(Volts));
    }
    // Setup idle mode.
    if (config.getIdleMode().isPresent())
    {
      sparkBaseConfig.idleMode(config.getIdleMode().get() == MotorMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
    }
    // Setup starting position
    if (config.getStartingPosition().isPresent())
    {
      sparkRelativeEncoder.setPosition(config.getStartingPosition().get().in(Rotations));
    }
    // Setup external encoder.
    if (config.getExternalEncoder().isPresent())
    {
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder instanceof SparkAbsoluteEncoder)
      {
        double absoluteEncoderConversionFactor = config.getExternalEncoderGearing().getRotorToMechanismRatio();
        sparkAbsoluteEncoder = Optional.of((SparkAbsoluteEncoder) externalEncoder);
        sparkBaseConfig.absoluteEncoder.positionConversionFactor(absoluteEncoderConversionFactor);
        sparkBaseConfig.absoluteEncoder.velocityConversionFactor(absoluteEncoderConversionFactor / 60);
        sparkBaseConfig.absoluteEncoder.inverted(config.getExternalEncoderInverted());

        if (RobotBase.isSimulation())
        {
          if (spark instanceof SparkMax)
          {
            sparkAbsoluteEncoderSim = Optional.of(new SparkAbsoluteEncoderSim((SparkMax) spark));
          } else if (spark instanceof SparkFlex)
          {
            sparkAbsoluteEncoderSim = Optional.of(new SparkAbsoluteEncoderSim((SparkFlex) spark));
          }
          if (config.getStartingPosition().isPresent())
          {
            sparkAbsoluteEncoderSim.ifPresent(enc -> enc.setPosition(config.getStartingPosition().get().in(Rotations)));
          }
        }
      } else
      {
        throw new IllegalArgumentException(
            "[ERROR] Unsupported external encoder: " + externalEncoder.getClass().getSimpleName());
      }

      // Set starting position if external encoder is empty.
      if (config.getStartingPosition().isEmpty())
      {
        sparkRelativeEncoder.setPosition(sparkAbsoluteEncoder.get().getPosition());
      }
    }

    // Configure follower motors
    if (config.getFollowers().isPresent())
    {
      for (Pair<Object, Boolean> follower : config.getFollowers().get())
      {
        if (follower.getFirst() instanceof SparkMax)
        {
          ((SparkMax) follower.getFirst()).configure(new SparkMaxConfig().follow(spark, follower.getSecond()),
                                                     ResetMode.kNoResetSafeParameters,
                                                     PersistMode.kPersistParameters);

        } else if (follower.getFirst() instanceof SparkFlex)
        {
          ((SparkFlex) follower.getFirst()).configure(new SparkFlexConfig().follow(spark, follower.getSecond()),
                                                      ResetMode.kNoResetSafeParameters,
                                                      PersistMode.kPersistParameters);

        } else
        {
          throw new IllegalArgumentException(
              "[ERROR] Unknown follower type: " + follower.getFirst().getClass().getSimpleName());
        }
      }
      config.clearFollowers();
    }

    if (config.getDiscontinuityPoint().isPresent())
    {
      throw new IllegalArgumentException(
          "[ERROR] Discontinuity point is not supported on Sparks, or we have not implemented this!");
    }

    if (config.getZeroOffset().isPresent())
    {
      DriverStation.reportError("[ERROR] Zero offset is not supported on Sparks, or we have not implemented this!",
                                false);
    }

    return configureSpark(() -> spark.configure(sparkBaseConfig,
                                                ResetMode.kNoResetSafeParameters,
                                                PersistMode.kPersistParameters));
  }

  @Override
  public double getDutyCycle()
  {
    return sparkSim.map(SparkSim::getAppliedOutput).orElseGet(spark::getAppliedOutput);
  }

  @Override
  public void setDutyCycle(double dutyCycle)
  {
    spark.set(dutyCycle);
    sparkSim.ifPresent(sparkSim1 -> sparkSim1.setAppliedOutput(dutyCycle));
  }

  @Override
  public Current getSupplyCurrent()
  {
    DriverStation.reportError("[WARNING] Supply currently not supported on Spark", true);
    return null;
  }

  @Override
  public Current getStatorCurrent()
  {
    return sparkSim.map(sim -> Amps.of(sim.getMotorCurrent())).orElseGet(() -> Amps.of(spark.getOutputCurrent()));
  }

  @Override
  public Voltage getVoltage()
  {
    return Volts.of(spark.getAppliedOutput() * spark.getBusVoltage());
  }

  @Override
  public void setVoltage(Voltage voltage)
  {
    spark.setVoltage(voltage);
  }

  @Override
  public DCMotor getDCMotor()
  {
    return motor;
  }

  @Override
  public LinearVelocity getMeasurementVelocity()
  {
    return config.convertFromMechanism(getMechanismVelocity());
  }

  @Override
  public Distance getMeasurementPosition()
  {
    return config.convertFromMechanism(getMechanismPosition());
  }

  @Override
  public AngularVelocity getMechanismVelocity()
  {
    if (sparkAbsoluteEncoder.isPresent() && config.getUseExternalFeedback())
    {
      return RotationsPerSecond.of(sparkAbsoluteEncoder.get().getVelocity());
    }
    return RotationsPerSecond.of(sparkRelativeEncoder.getVelocity());
  }

  @Override
  public Angle getMechanismPosition()
  {
    Angle pos = Rotations.of(sparkRelativeEncoder.getPosition());
    if (sparkAbsoluteEncoder.isPresent() && config.getUseExternalFeedback())
    {
      pos = Rotations.of(sparkAbsoluteEncoder.get().getPosition());
    }
    if (config.getZeroOffset().isPresent())
    {
      pos = pos.minus(config.getZeroOffset().get());
    }
    return pos;
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    return RotationsPerSecond.of(getMechanismPosition().in(Rotations) * config.getGearing().getMechanismToRotorRatio());
  }

  @Override
  public Angle getRotorPosition()
  {
    return Rotations.of(getMechanismPosition().in(Rotations) * config.getGearing().getMechanismToRotorRatio());
  }

  @Override
  public Temperature getTemperature()
  {
    return Celsius.of(spark.getMotorTemperature());
  }

  @Override
  public SmartMotorControllerConfig getConfig()
  {
    return config;
  }

  @Override
  public Object getMotorController()
  {
    return spark;
  }

  @Override
  public Object getMotorControllerConfig()
  {
    return sparkBaseConfig;
  }
}
