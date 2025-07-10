package yams.motorcontrollers.local;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.Optional;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public class NovaWrapper extends SmartMotorController
{

  /**
   * Thrifty Nova controller.
   */
  private final ThriftyNova          m_nova;
  /**
   * Motor characteristics controlled by the {@link ThriftyNova}.
   */
  private final DCMotor              m_motor;
  /**
   * Sim for ThriftyNova's.
   */
  private       Optional<DCMotorSim> m_sim = Optional.empty();

  /**
   * Construct the {@link SmartMotorController} for the generic {@link ThriftyNova} controller.
   * This constructor is only used to create a ThriftyNova for reflection.
   * @param id
   */
  public NovaWrapper(int id) {
    m_nova = new ThriftyNova(id);
    m_motor = DCMotor.getNEO(1); // Default to NEO motor
  }

  /**
   * Construct the Nova Wrapper for the generic {@link SmartMotorController}.
   *
   * @param controller {@link ThriftyNova} to use.
   * @param motor      {@link DCMotor} connected to the {@link ThriftyNova}.
   * @param config     {@link SmartMotorControllerConfig} to apply to the {@link ThriftyNova}.
   */
  public NovaWrapper(ThriftyNova controller, DCMotor motor, SmartMotorControllerConfig config)
  {
    m_nova = controller;
    this.m_motor = motor;
    this.config = config;
  }

  @Override
  public void setupSimulation()
  {
    if (RobotBase.isSimulation())
    {
      m_sim = Optional.of(new DCMotorSim(LinearSystemId.createDCMotorSystem(m_motor,
                                                                            0.001,
                                                                            config.getGearing()
                                                                                  .getRotorToMechanismRatio()),
                                         m_motor,
                                         1.0 / 1024.0));
    }
  }

  @Override
  public void seedRelativeEncoder()
  {
    DriverStation.reportWarning("[WARNING] NovaWrapper.seedRelativeEncoder() is not supported", false);
  }

  @Override
  public void synchronizeRelativeEncoder()
  {
    DriverStation.reportWarning("[WARNING] NovaWrapper.synchronizeRelativeEncoder() is not supported on ThriftyNova's.",
                                false);
  }

  @Override
  public void simIterate(AngularVelocity mechanismVelocity)
  {
    if (RobotBase.isSimulation())
    {
      m_sim.ifPresent(sim -> {
        sim.setAngularVelocity(mechanismVelocity.in(RadiansPerSecond));
        sim.update(config.getClosedLoopControlPeriod().in(Seconds));
      });
    }
  }

  @Override
  public void setEncoderVelocity(AngularVelocity velocity)
  {
//    m_sim.ifPresent(dcMotorSim -> dcMotorSim.setAngularVelocity(velocity.in(RadiansPerSecond)));
    DriverStation.reportWarning("[WARNING] NovaWrapper.setEncoderVelocity() is not supported on ThriftyNova's.", false);
  }

  @Override
  public void setEncoderVelocity(LinearVelocity velocity)
  {
    DriverStation.reportWarning("[WARNING] NovaWrapper.setEncoderVelocity() is not supported on ThriftyNova's.", false);
  }

  @Override
  public void setEncoderPosition(Angle angle)
  {
    m_nova.setEncoderPosition(angle.in(Rotations));
    m_sim.ifPresent(sim -> sim.setAngle(angle.in(Rotations)));
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

    // Ramp rates
    m_nova.setRampUp(config.getClosedLoopRampRate().in(Seconds));
    m_nova.setRampDown(config.getClosedLoopRampRate().in(Seconds));
    if (config.getOpenLoopRampRate().gt(Seconds.of(0)))
    {
      throw new IllegalArgumentException(
          "[Error] ThriftyNova does not support separate closed loop and open loop ramp rates, using the SmartMotorControllerConfig.withClosedLoopRampRate() as both.");
    }

    // Inversions
    m_nova.setInverted(config.getMotorInverted());
    if (config.getEncoderInverted())
    {
      throw new IllegalArgumentException("[ERROR] ThriftyNova does not support encoder inversions.");
    }

    // Current limits
    if (config.getSupplyStallCurrentLimit().isPresent())
    {
      m_nova.setMaxCurrent(CurrentType.SUPPLY, config.getSupplyStallCurrentLimit().getAsInt());
    }
    if (config.getStatorStallCurrentLimit().isPresent())
    {
      m_nova.setMaxCurrent(CurrentType.STATOR, config.getStatorStallCurrentLimit().getAsInt());
    }

    // Voltage Compensation
    if (config.getVoltageCompensation().isPresent())
    {
      m_nova.setVoltageCompensation(config.getVoltageCompensation().get().in(Volts));
    }

    // Setup idle mode.
    if (config.getIdleMode().isPresent())
    {
      m_nova.setBrakeMode(config.getIdleMode().get() == MotorMode.BRAKE);
    }

    // Starting Position
    if (config.getStartingPosition().isPresent())
    {
      setEncoderPosition(config.getStartingPosition().get());
    }

    // External Encoder
    if (config.getExternalEncoder().isPresent())
    {
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder == EncoderType.QUAD)
      {
        m_nova.useEncoderType(EncoderType.QUAD);
      } else if (externalEncoder == EncoderType.ABS)
      {
        m_nova.useEncoderType(EncoderType.ABS);
      } else
      {
        throw new IllegalArgumentException(
            "[ERROR] Unsupported external encoder: " + externalEncoder.getClass().getSimpleName() +
            ".\n\tPlease use an `EncoderType` instead.");
      }

      if (config.getStartingPosition().isEmpty())
      {
        if (externalEncoder == EncoderType.ABS)
        {
          m_nova.setEncoderPosition(m_nova.getPositionAbs());
        }
      }
    }

    if (config.getFollowers().isPresent())
    {
      for (Pair<Object, Boolean> follower : config.getFollowers().get())
      {
        if (follower.getFirst() instanceof ThriftyNova)
        {
          ((ThriftyNova) follower.getFirst()).follow(m_nova.getID());
          ((ThriftyNova) follower.getFirst()).setInverted(follower.getSecond());
        } else
        {
          throw new IllegalArgumentException(
              "[ERROR] Unknown follower type: " + follower.getFirst().getClass().getSimpleName());
        }
      }
    }

    if (config.getZeroOffset().isPresent())
    {
      DriverStation.reportError("[ERROR] ThriftyNova does not support zero offset, or we have not implemented this.",
                                true);
    }

    if (config.getDiscontinuityPoint().isPresent())
    {
      throw new IllegalArgumentException(
          "[ERROR] ThriftyNova does not support discontinuity points, or we have not implemented this.");
    }

    return true;
  }

  @Override
  public double getDutyCycle()
  {
    return m_sim.map(dcMotorSim -> dcMotorSim.getInputVoltage() / RoboRioSim.getVInVoltage())
                .orElseGet(m_nova::get);
  }

  @Override
  public void setDutyCycle(double dutyCycle)
  {
    m_sim.ifPresent(sim -> sim.setInputVoltage(dutyCycle * RoboRioSim.getVInVoltage()));
    m_nova.set(dutyCycle);
  }

  @Override
  public Current getSupplyCurrent()
  {
    if (m_sim.isPresent())
    {
      return Amps.of(RoboRioSim.getVInCurrent());
    }
    return Amps.of(m_nova.getSupplyCurrent());
  }

  @Override
  public Current getStatorCurrent()
  {
    return Amps.of(m_sim.map(DCMotorSim::getCurrentDrawAmps).orElseGet(m_nova::getStatorCurrent));
  }

  @Override
  public Voltage getVoltage()
  {
    return Volts.of(m_sim.map(DCMotorSim::getInputVoltage).orElseGet(m_nova::getVoltage));
  }

  @Override
  public void setVoltage(Voltage voltage)
  {
    m_sim.ifPresent(dcMotorSim -> dcMotorSim.setInputVoltage(voltage.in(Volts)));
    m_nova.setVoltage(voltage);
  }

  @Override
  public DCMotor getDCMotor()
  {
    return m_motor;
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
    if (m_sim.isPresent())
    {
      return m_sim.get().getAngularVelocity();
    }
    if (config.getUseExternalFeedback() && config.getExternalEncoder().isPresent())
    {
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder == EncoderType.ABS)
      {
        // Do nothing since attached absolute encoders do not give their velocity.
        // There should be an alert thrown here; but Alerts are not thread-safe.
      } else if (externalEncoder == EncoderType.QUAD)
      {
        // There should be an alert thrown here; but Alerts are not thread-safe.
        return RotationsPerSecond.of(
            m_nova.getVelocityQuad() * config.getExternalEncoderGearing().getRotorToMechanismRatio());
      }
    }
    return getRotorVelocity().times(config.getGearing().getRotorToMechanismRatio());
  }

  @Override
  public Angle getMechanismPosition()
  {
    if (m_sim.isPresent())
    {
      return m_sim.get().getAngularPosition();
    }
    if (config.getUseExternalFeedback() && config.getExternalEncoder().isPresent())
    {
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder == EncoderType.ABS)
      {
        return Rotations.of(m_nova.getPositionAbs() * config.getExternalEncoderGearing().getRotorToMechanismRatio());
      } else if (externalEncoder == EncoderType.QUAD)
      {
        return Rotations.of(m_nova.getPositionQuad() * config.getExternalEncoderGearing().getRotorToMechanismRatio());
      }
    }
    return getRotorPosition().times(config.getGearing().getRotorToMechanismRatio());
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    if (RobotBase.isSimulation() && m_sim.isPresent())
    {
      return m_sim.get().getAngularVelocity().times(config.getGearing().getMechanismToRotorRatio());
    }
    return RotationsPerSecond.of(m_nova.getVelocity());
  }

  @Override
  public Angle getRotorPosition()
  {
    if (RobotBase.isSimulation() && m_sim.isPresent())
    {
      return m_sim.get().getAngularPosition().times(config.getGearing().getMechanismToRotorRatio());
    }
    return Rotations.of(m_nova.getPosition());
  }

  @Override
  public Temperature getTemperature()
  {
    return Celsius.of(m_nova.getTemperature());
  }

  @Override
  public SmartMotorControllerConfig getConfig()
  {
    return config;
  }

  @Override
  public Object getMotorController()
  {
    return m_nova;
  }

  @Override
  public Object getMotorControllerConfig()
  {
    DriverStation.reportWarning(
        "[WARNING] Thrifty Nova's have no configuration class, returning the ThriftyNova Object.",
        true);
    return m_nova;
  }
}
