// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

    double sensorToMeterCoefficientInside = 1 * 0.05 * 0.03 * Math.PI;
    double sensorToMeterCoefficientOutside = 1 * 0.05 * 0.027 * Math.PI;

    // Units are meters!!
    final double outsideWinchMaxHeight = 0.48; // Limits when the arms are in the opened (/) position
    final double insideWinchMaxHeight = 0.70; // Limits when the arms are in the opened (/) position
    final double outsideWinchMaxHeightClosed = 0.406; // Limits when the arms are in the closed (|) position
    final double insideWinchMaxHeightClosed = 0.516; // Limits when the arms are in the closed (|) position
    final double outsideWinchMinHeight = 0.0075;
    final double insideWinchMinHeight = 0.0075;

    private double outsideMaxHeight;
    private double insideMaxHeight;

    boolean useLimits = true;

    private boolean isResetingOutsideArm = false;
    private boolean isResetingInsideArm = false;

    private int frameSinceInsideClose = 500;
    private int framesSinceAutoOpen = 500;

    private boolean beenEnabled = false;

    /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {
        outsideWinch = new CANSparkMax(5, MotorType.kBrushless);
        insideWinch = new CANSparkMax(6, MotorType.kBrushless);

        insideWinch.setInverted(true);

        outsideEncoder = outsideWinch.getEncoder();
        insideEncoder = insideWinch.getEncoder();
        outsideEncoder.setPosition(0);
        insideEncoder.setPosition(0);

        outsideSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);
        insideSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 8);
        insideBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 4);

        outsideMaxHeight = outsideWinchMaxHeightClosed;
        insideMaxHeight = insideWinchMaxHeightClosed;

        outsideWinch.setIdleMode(IdleMode.kBrake);
        insideWinch.setIdleMode(IdleMode.kBrake);

        setOutsideSolenoid(false);
        setInsideSolenoid(false);

    }

    public void onEnable() {
        if (!beenEnabled) {
            // setOutsideSolenoid(false);
            // setInsideSolenoid(false);
        }
        beenEnabled = true;
    }

    public void setOutsideSolenoid(boolean open) {
        if (open) {
            outsideSolenoid.set(Value.kForward);

            outsideMaxHeight = outsideWinchMaxHeight;
        } else {
            outsideSolenoid.set(Value.kReverse);

            outsideMaxHeight = outsideWinchMaxHeightClosed;

        }
        ;
    }

    public void setInsideSolenoid(boolean open) {
        if (open) {
            insideSolenoid.set(Value.kForward);
            frameSinceInsideClose = Integer.MIN_VALUE;
            insideMaxHeight = insideWinchMaxHeight;
        } else {
            insideSolenoid.set(Value.kReverse);
            frameSinceInsideClose = 0;
            insideMaxHeight = insideWinchMaxHeightClosed;
        }
    }

    public void toggleOutsideSolenoid() {
        if (outsideSolenoid.get() == Value.kReverse) {
            outsideSolenoid.set(Value.kForward);

            outsideMaxHeight = outsideWinchMaxHeight;
        } else {
            outsideSolenoid.set(Value.kReverse);
            outsideMaxHeight = outsideWinchMaxHeightClosed;
        }
    }

    public void toggleInsideSolenoid() {
        if (insideSolenoid.get() == Value.kReverse) {
            insideSolenoid.set(Value.kForward);
            frameSinceInsideClose = Integer.MIN_VALUE;
            insideMaxHeight = insideWinchMaxHeight;
        } else {
            insideSolenoid.set(Value.kReverse);
            frameSinceInsideClose = 0;
            insideMaxHeight = insideWinchMaxHeightClosed;
        }
    }

    public void moveOutsideArm(double value) {
        double armExtension = outsideEncoder.getPosition() * sensorToMeterCoefficientOutside;
        if (!useLimits) {
            if (armExtension < outsideMaxHeight && armExtension > outsideWinchMinHeight) {
                outsideWinch.set(value);
            } else if (value < 0 && armExtension > outsideMaxHeight) {
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

        double armExtension = insideEncoder.getPosition() * sensorToMeterCoefficientInside;
        if (!useLimits) {
            if (armExtension < insideMaxHeight && armExtension > insideWinchMinHeight) {
                insideWinch.set(value);
            } else if (value < 0 && armExtension > insideMaxHeight) {
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

    public void startResetOutsideArmsLength() {
        isResetingOutsideArm = true;
        outsideWinch.set(-0.1);
    }

    public void stopResetOutsideArm(boolean finished) {
        isResetingOutsideArm = false;
        outsideWinch.set(0);
        if (finished) {
            outsideEncoder.setPosition(0);
        }
    }

    public void startOutsideOpen() {
        framesSinceAutoOpen = 0;
        outsideWinch.set(0.5);
    }

    void stopOutsideOpen() {
        moveOutsideArm(0);
        toggleOutsideSolenoid();
    }

    public void startResetInsideArmsLength() {
        insideBrake.set(Value.kReverse);
        isResetingInsideArm = true;
        insideWinch.set(-0.1);
    }

    public void stopResetInsideArm(boolean finished) {
        insideBrake.set(Value.kForward);
        isResetingInsideArm = false;
        insideWinch.set(0);
        if (finished)
            insideEncoder.setPosition(0);
    }

    public void checkForExtensionOutside() {
        if (outsideEncoder.getPosition() * sensorToMeterCoefficientOutside > 0.03 && !isResetingOutsideArm) {
            startResetOutsideArmsLength();
        } else if (!isResetingOutsideArm) {
            moveOutsideArm(0);
        }
    }

    @Override
    public void periodic() {

        if (framesSinceAutoOpen == 15) {
            outsideWinch.set(0);
            toggleOutsideSolenoid();
        }

        if (isResetingOutsideArm && outsideWinch.getOutputCurrent() > 30) {
            stopResetOutsideArm(true);
        }
        if (isResetingInsideArm && insideWinch.getOutputCurrent() > 25) {
            stopResetInsideArm(true);
        }


        if (frameSinceInsideClose == 12) {
            insideSolenoid.set(Value.kForward);
        } else if (frameSinceInsideClose == 18) {
            insideSolenoid.set(Value.kReverse);
        }

        SmartDashboard.putString("Outside Arms Extension",
                String.format("%2.3f", outsideEncoder.getPosition() * sensorToMeterCoefficientOutside));
        SmartDashboard.putString("Inside Arms Extension", String.format("%2.3f", insideEncoder.getPosition() * sensorToMeterCoefficientInside));
        SmartDashboard.putBoolean("Limits", !useLimits);
        frameSinceInsideClose++;
        framesSinceAutoOpen++;
    }
}
