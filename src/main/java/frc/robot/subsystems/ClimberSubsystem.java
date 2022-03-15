// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
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
    double outsideWinchMaxHeight = 0.54;
    double insideWinchMaxHeight = 0.55;
    double outsideWinchMaxHeightClosed = 0.4;
    double insideWinchMaxHeightClosed = 0.4;
    double outsideWinchMinHeight = 0.005;
    double insideWinchMinHeight = 0.005;

    private double outsideMaxHeight;
    private double insideMaxHeight;

    boolean useLimits = true;

    private boolean isResetingOutsideArm = false;
    private boolean isResetingInsideArm = false;

    private int frameSinceOutsideOpen;
    private int frameSinceOutsideClose;
    private int frameSinceInsideOpen;
    private int frameSinceInsideClose;
    

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
        
        // setOutsideSolenoid(false);
        // setInsideSolenoid(false);
        outsideMaxHeight = outsideWinchMaxHeightClosed;
        insideMaxHeight = insideWinchMaxHeightClosed;

        outsideWinch.setIdleMode(IdleMode.kBrake);
        insideWinch.setIdleMode(IdleMode.kBrake); 
        
    }

    public void setOutsideSolenoid(boolean open) {
        if (open) {
            outsideSolenoid.set(Value.kForward);
            frameSinceOutsideOpen = 0;
            frameSinceOutsideClose = Integer.MIN_VALUE;
            outsideMaxHeight = outsideWinchMaxHeight;
        } else {
            outsideSolenoid.set(Value.kReverse);
            frameSinceOutsideClose = 0;
            frameSinceOutsideOpen = Integer.MIN_VALUE;
            outsideMaxHeight = outsideWinchMaxHeightClosed;
            
        }
        ;
    }

    public void setInsideSolenoid(boolean open) {
        if (open) {
            insideSolenoid.set(Value.kForward);
            frameSinceInsideOpen = 0;
            frameSinceInsideClose = Integer.MIN_VALUE;
            insideMaxHeight = insideWinchMaxHeight;
        } else {
            insideSolenoid.set(Value.kReverse);
            frameSinceInsideClose = 0;
            frameSinceInsideOpen = Integer.MIN_VALUE;
            insideMaxHeight = insideWinchMaxHeightClosed;
        }
    }

    public void toggleOutsideSolenoid() {
        if (outsideSolenoid.get() == Value.kReverse) {
            outsideSolenoid.set(Value.kForward);
            frameSinceOutsideOpen = 0;
            frameSinceOutsideClose = Integer.MIN_VALUE;
            outsideMaxHeight = outsideWinchMaxHeight;
        } else {
            outsideSolenoid.set(Value.kReverse);
            frameSinceOutsideClose = 0;
            frameSinceOutsideOpen = Integer.MIN_VALUE;
            outsideMaxHeight = outsideWinchMaxHeightClosed;
        }
    }

    public void toggleInsideSolenoid() {
        if (insideSolenoid.get() == Value.kReverse) {
            insideSolenoid.set(Value.kForward);
            frameSinceInsideOpen = 0;
            frameSinceInsideClose = Integer.MIN_VALUE;
            insideMaxHeight = insideWinchMaxHeight;
        } else {
            insideSolenoid.set(Value.kReverse);
            frameSinceInsideClose = 0;
            frameSinceInsideOpen = Integer.MIN_VALUE;
            insideMaxHeight = insideWinchMaxHeightClosed;
        }
    }

    public void moveOutsideArm(double value) {
        double armExtension = outsideEncoder.getPosition() * sensorToMeterCoefficient;
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

        double armExtension = insideEncoder.getPosition() * sensorToMeterCoefficient;
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
        if (finished)
        {
            outsideEncoder.setPosition(0);
        }
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

    @Override
    public void periodic() {
        if (isResetingOutsideArm && outsideWinch.getOutputCurrent() > 21) {
            stopResetOutsideArm(true);
        }
        if (isResetingInsideArm && insideWinch.getOutputCurrent() > 21) {
            stopResetInsideArm(true);
        }

        if (frameSinceOutsideOpen == 8){
            outsideSolenoid.set(Value.kReverse);
        } else if (frameSinceOutsideOpen == 15){
            outsideSolenoid.set(Value.kForward);
        }
        if (frameSinceOutsideClose == 12){
            outsideSolenoid.set(Value.kForward);
        } else if (frameSinceOutsideClose == 18){
            outsideSolenoid.set(Value.kReverse);
        }
        if (frameSinceInsideOpen == 10){
            insideSolenoid.set(Value.kReverse);
        } else if (frameSinceInsideOpen == 15){
            insideSolenoid.set(Value.kForward);
        }
        if (frameSinceInsideClose == 12){
            insideSolenoid.set(Value.kForward);
        } else if (frameSinceInsideClose == 18){
            insideSolenoid.set(Value.kReverse);
        }


        SmartDashboard.putNumber("Outside Arms Extension", outsideEncoder.getPosition() * sensorToMeterCoefficient);
        SmartDashboard.putNumber("Inside Arms Extension", insideEncoder.getPosition() * sensorToMeterCoefficient);
        frameSinceOutsideOpen++;
        frameSinceOutsideClose++;
        frameSinceInsideOpen++;
        frameSinceInsideClose++;
    }
}
