// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package yams.config;

/** Add your docs here. */
public class UnitValueJson {
    public String uom;
    public double val;

    public UnitValueJson() {}
    
    public UnitValueJson(double val, String uom) {
        this.val = val;
        this.uom = uom;
    }

    public double getMagnitude() {
        return val;
    }

    public String getUnit() {
        return uom;
    }
}
