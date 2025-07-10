// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.config.ArmParser;
import yams.config.ElevatorParser;
import yams.config.PivotParser;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.positional.Pivot;

public class ConfiguredMechanisms extends SubsystemBase {
  Elevator elevator;
  Arm arm; // Assuming you have an Arm subsystem, otherwise remove this line
  Pivot pivot; // Assuming you have a Pivot subsystem, otherwise remove this line

  /** Creates a new ConfiguredElevator. */
  public ConfiguredMechanisms() {
    elevator = ElevatorParser.parse("mechanisms", "yams_elevator.json", this);
    arm = ArmParser.parse("mechanisms", "yams_arm.json", this);
    pivot = PivotParser.parse("mechanisms", "yams_pivot.json", this);
  }

  public void periodic()
  {
    elevator.updateTelemetry();
    arm.updateTelemetry();
    pivot.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    elevator.simIterate();
    arm.simIterate();
    pivot.simIterate();
  }

  public Command elevCmd(double dutycycle)
  {
    return elevator.set(dutycycle);
  }

  public Command setHeight(Distance height)
  {
    return elevator.setHeight(height);
  }

  public Command sysIdElevator()
  {
    return elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
  }

  public Command armCmd(double dutycycle)
  {
    return arm.set(dutycycle);
  }

  public Command sysIdArm()
  {
    return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setArmAngle(Angle angle)
  {
    return arm.setAngle(angle);
  }

  public Command turretCmd(double dutycycle)
  {
    return pivot.set(dutycycle);
  }

  public Command sysIdTurret()
  {
    return pivot.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setPivotAngle(Angle angle)
  {
    return pivot.setAngle(angle);
  }
}
