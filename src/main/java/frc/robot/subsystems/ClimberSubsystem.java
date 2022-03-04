// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    CANSparkMax outsideWinch;
    CANSparkMax insideWinch;

    RelativeEncoder outsideEncoder;
    RelativeEncoder insideEncoder;

    DoubleSolenoid outsideSolenoid;
    DoubleSolenoid insideSolenoid;
    DoubleSolenoid insideBrake;

    double sensorToMeterCoefficient = 1 * 0.05 * 0.03 * Math.PI;
    // Units are meters!!
    double outsideWinchMaxHeight = 0.6;
    double insideWinchMaxHeight = 0.6;
    double outsideWinchMinHeight = 0.02;
    double insideWinchMinHeight = 0.02;

    boolean useLimits = true;

    /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {
        outsideWinch = new CANSparkMax(5, MotorType.kBrushless);
        insideWinch = new CANSparkMax(6, MotorType.kBrushless);

        insideWinch.setInverted(true);

        outsideEncoder = outsideWinch.getEncoder();
        insideEncoder = insideWinch.getEncoder();
        outsideEncoder.setPosition(0);
        insideEncoder.setPosition(0);

        outsideSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 8);
        insideSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);
        insideBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 4);
    }

    public void setOutsideSolenoid(boolean open) {
        if (open) {
            outsideSolenoid.set(Value.kForward);
        } else {
            outsideSolenoid.set(Value.kReverse);
        }
    }

    public void setInsideSolenoid(boolean open) {
        if (open) {
            insideSolenoid.set(Value.kForward);
        } else {
            insideSolenoid.set(Value.kReverse);
        }
    }

    public void toggleOutsideSolenoid() {
        if (outsideSolenoid.get() == Value.kReverse) {
            outsideSolenoid.set(Value.kForward);
        } else {
            outsideSolenoid.set(Value.kReverse);
        }
    }

    public void toggleInsideSolenoid() {
        if (insideSolenoid.get() == Value.kReverse) {
            insideSolenoid.set(Value.kForward);
        } else {
            insideSolenoid.set(Value.kReverse);
        }
    }

    public void moveOutsideArm(double value) {
        double armExtension = outsideEncoder.getPosition() * sensorToMeterCoefficient;
        if (!useLimits) {
            if (armExtension < outsideWinchMaxHeight && armExtension > outsideWinchMinHeight) {
                outsideWinch.set(value);
            } else if (value < 0 && armExtension > outsideWinchMaxHeight) {
                outsideWinch.set(value);
            } else if (armExtension < outsideWinchMinHeight && value > 0) {
                outsideWinch.set(value);
            } else {
                outsideWinch.set(0);
            }
        } else {
            outsideWinch.set(value);
        }
    }

    public void moveInsideArm(double value) {
        if (value == 0) {
            insideBrake.set(Value.kForward);
        } else {
            insideBrake.set(Value.kReverse);
        }
        
        double armExtension = insideEncoder.getPosition() * sensorToMeterCoefficient;
        System.out.println("Erection length: " + armExtension);
        if (!useLimits) {
            if (armExtension < insideWinchMaxHeight && armExtension > insideWinchMinHeight) {
                insideWinch.set(value);
            } else if (value < 0 && armExtension > insideWinchMaxHeight) {
                insideWinch.set(value);
            } else if (armExtension < insideWinchMinHeight && value > 0) {
                insideWinch.set(value);
            } else {
                insideWinch.set(0);
            }
        } else {
            insideWinch.set(value);
        }
    }

    public void setBrake(boolean value) {
        insideBrake.set(value ? Value.kForward : Value.kReverse);
    }

    public void setLimitsEnabled(boolean enabled) {
        useLimits = enabled;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Outside Arms Extension", outsideEncoder.getPosition() * sensorToMeterCoefficient);
        SmartDashboard.putNumber("Inside Arms Extension", insideEncoder.getPosition() * sensorToMeterCoefficient);
    }
}
