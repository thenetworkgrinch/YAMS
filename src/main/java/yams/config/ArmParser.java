// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package yams.config;

import java.io.File;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.positional.Arm;

/** Add your docs here. */
public class ArmParser {
    public static Arm parse(String subDirectory, String filename, SubsystemBase system) 
    {
        try {
            File directory = new File(Filesystem.getDeployDirectory(), subDirectory);
            DeviceConfigReader.checkDirectory(directory);
            File deviceFile = new File(directory, filename);
            YamsArmConfigurationJson yamsArmConfigurationJson = new ObjectMapper().readValue(deviceFile,
                    YamsArmConfigurationJson.class);
            return yamsArmConfigurationJson.configure(system);
        } catch (Exception e) {
            System.out.println("Error reading device configuration: " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }
}
