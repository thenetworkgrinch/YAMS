package yams.config;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface DeviceConfiguration {
  public Object configure(SubsystemBase deviceHandler);
}
