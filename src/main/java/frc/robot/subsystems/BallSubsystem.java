// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallSubsystem extends SubsystemBase {

    TalonFX shooterFalcon;
    TalonSRX conveyor775;
    TalonSRX intake775;
    DoubleSolenoid intakeSolenoid;
    PowerDistribution pdp;
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;
    Color ledColor;
    static final Color disableColor = new Color(1.0f * 0.3, 0.27058825f * 0.3, 0.0f);
    static final Color blueColor = Color.kBlue;
    static final Color redColor = Color.kRed;

    int falconUnitsTargetVelocity;
    boolean shooterReachedSpeed;
    int conveyorReverseTimer;
    public final static double falconToRPMCoefficient = 1 * 600.0 / 2048.0 * 24.0 / 34.0;
    boolean ballAtBarrel = false;
    boolean intakeOpen = false;
    int framesSinceIntakeOpen = 0;
    int framesSinceIntakeClosed = 0;
    boolean shooterWarming = false;
    int locationInStrip = 0;
    int ledBlinkCounter = 0;

    public BallSubsystem() {
        shooterFalcon = new TalonFX(Constants.SHOOTER_TALONFX_MOTOR);
        conveyor775 = new TalonSRX(Constants.CONVEYOR_TALONSRX_MOTOR);
        intake775 = new TalonSRX(Constants.INTAKE_TALONSRX_MOTOR);

        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
        pdp = new PowerDistribution(0, ModuleType.kCTRE);

        led = new AddressableLED(5);
        ledBuffer = new AddressableLEDBuffer(22);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }

        led.setData(ledBuffer);
        DrivetrainSubsystem.updateFalconPID(Constants.SHOOTER_TALONFX_MOTOR, 0.07, 0, 0,
                0.053, NeutralMode.Coast);
        setShooterSpeed(Constants.SHOOTER_FLYWHEEL_RPM_HIGH_GOAL);
        closeIntake();
        stopShooter();
    }

    public void openIntake() {
        intakeOpen = true;
        ballAtBarrel = true;
        conveyor775.set(ControlMode.PercentOutput, Constants.CONVEYOR_SPEED_PERCENT_INTAKING);
        intakeSolenoid.set(Value.kForward);
        framesSinceIntakeOpen = 0;
        framesSinceIntakeClosed = Integer.MIN_VALUE;
        shooterFalcon.set(ControlMode.PercentOutput, -0.4);
    }

    public void closeIntake(boolean stopConveyor) {
        intake775.set(ControlMode.PercentOutput, 0);
        intakeSolenoid.set(Value.kReverse);
        framesSinceIntakeOpen = Integer.MIN_VALUE;
        framesSinceIntakeClosed = 0;
        intakeOpen = false;
        if (stopConveyor) {
            conveyor775.set(ControlMode.PercentOutput, 0);
        }
    }

    public void closeIntake() {
        boolean stopConveyor = true;
        intake775.set(ControlMode.PercentOutput, 0);
        intakeSolenoid.set(Value.kReverse);
        framesSinceIntakeOpen = Integer.MIN_VALUE;
        framesSinceIntakeClosed = 0;
        intakeOpen = false;
        if (stopConveyor) {
            conveyor775.set(ControlMode.PercentOutput, 0);
        }
    }

    public void prepareForShootingInit() {
        if (intakeOpen)
            closeIntake(false);

        shooterWarming = true;
        conveyor775.set(ControlMode.PercentOutput, Constants.CONVEYOR_SPEED_PERCENT_REVERSE);
        conveyorReverseTimer = Constants.CONVEYER_REVERSE_DURATION_FRAMES;
        shooterFalcon.set(ControlMode.PercentOutput, -0.4);
    }

    public void onEnable() {
        if (DriverStation.getAlliance() == Alliance.Red) {
            ledColor = redColor;
        } else {
            ledColor = blueColor;
        }
    }

    public void onDisable() {
        ledColor = disableColor;
    }

    public void setShooterSpeed(int rpmTarget) {
        falconUnitsTargetVelocity = (int) (rpmTarget / falconToRPMCoefficient);
        if (shooterWarming && conveyorReverseTimer < 0) {
            shooterFalcon.set(ControlMode.Velocity, falconUnitsTargetVelocity);
        }
        System.out.println(rpmTarget);
        SmartDashboard.putNumber("Shooter Target", rpmTarget);
    }

    public void shoot() {
        if (shooterReachedSpeed && !ballAtBarrel) {
            conveyor775.set(ControlMode.PercentOutput, Constants.CONVEYOR_SPEED_PERCENT_SHOOTING);
        } else if (!shooterWarming) {
            prepareForShootingInit();
        }
    }

    public void stopShooter() {
        if (!intakeOpen)
            conveyor775.set(ControlMode.PercentOutput, 0);

        shooterWarming = false;
        shooterFalcon.set(ControlMode.PercentOutput, 0);
        conveyorReverseTimer = -1;
    }

    public void startSpittingShooter() {
        stopShooter();
        conveyor775.set(ControlMode.PercentOutput, -1);
        shooterFalcon.set(ControlMode.PercentOutput, 0.2);

        framesSinceIntakeClosed = Integer.MIN_VALUE;
        framesSinceIntakeOpen = Integer.MIN_VALUE;
    }

    public void stopSpittingShooter() {
        stopShooter();
        if (intakeOpen) {
            framesSinceIntakeOpen = 200;
        } else {
            framesSinceIntakeClosed = 200;
        }
    }

    public void startReversingIntake() {
        intake775.set(ControlMode.PercentOutput, 0.4);
    }

    public void stopReversingIntake() {
        if (intakeOpen) {
            intake775.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED_PERCENT);
        } else {
            intake775.set(ControlMode.PercentOutput, 0);
        }
    }

    private void setLED() {
        if (shooterWarming) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {

                if (i == locationInStrip % ledBuffer.getLength()) {
                    ledBuffer.setLED(i, ledColor);
                } else if (i == (locationInStrip + 1) % ledBuffer.getLength()) {
                    ledBuffer.setLED(i, ledColor);
                } else if (i == (locationInStrip + 2) % ledBuffer.getLength()) {
                    ledBuffer.setLED(i, ledColor);
                } else if (i == (locationInStrip + 3) % ledBuffer.getLength()) {
                    ledBuffer.setLED(i, ledColor);
                } else if (i == (locationInStrip + 4) % ledBuffer.getLength()) {
                    ledBuffer.setLED(i, ledColor);
                } else if (i == (locationInStrip + 5) % ledBuffer.getLength()) {
                    ledBuffer.setLED(i, ledColor);
                } else {
                    ledBuffer.setLED(i, Color.kBlack);
                }

            }
            locationInStrip += 2;

            // ledBuffer.setLED(locationInStrip, ledColor);
            // if (locationInStrip == 0) {
            // ledBuffer.setLED(ledBuffer.getLength() - 3, Color.kBlack);
            // } else if (locationInStrip == 1){
            // ledBuffer.setLED(ledBuffer.getLength() - 2, Color.kBlack);
            // } else if (locationInStrip == 2){
            // ledBuffer.setLED(ledBuffer.getLength() - 1, Color.kBlack);
            // } else {
            // ledBuffer.setLED(locationInStrip - 3, Color.kBlack);
            // }
            // locationInStrip++;
            // if (locationInStrip == ledBuffer.getLength()) {
            // locationInStrip = 0;
            // }
        } else if (intakeOpen) {
            if (ledBlinkCounter < 15) {
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setLED(i, ledColor);
                }
            } else {
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setLED(i, Color.kBlack);
                }
            }
            ledBlinkCounter++;
            if (ledBlinkCounter == 30) {
                ledBlinkCounter = 0;
            }
        } else {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, ledColor);
            }
        }
        led.setData(ledBuffer);
    }

    @Override
    public void periodic() {

        // Checking if reached speed
        if (Math.abs(shooterFalcon.getSelectedSensorVelocity()
                - falconUnitsTargetVelocity) < Constants.SHOOTER_FLYWHEEL_RPM_ERROR / falconToRPMCoefficient) {
            shooterReachedSpeed = true;
        } else {
            shooterReachedSpeed = false;
        }

        // RPM on dashboard
        SmartDashboard.putNumber("RPM", shooterFalcon.getSelectedSensorVelocity() * falconToRPMCoefficient);

        // Waiting to spin the intake
        if (framesSinceIntakeOpen == 20) {
            intake775.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED_PERCENT);
        }

        if (framesSinceIntakeClosed == 50) {
            conveyor775.set(ControlMode.PercentOutput, 0);
            if (!shooterWarming) {
                shooterFalcon.set(ControlMode.PercentOutput, 0);
            }
        }

        // Soften intake open and close
        if (framesSinceIntakeOpen == 8) {
            intakeSolenoid.set(Value.kReverse);
        } else if (framesSinceIntakeOpen == 18) {
            intakeSolenoid.set(Value.kForward);
        }

        if (framesSinceIntakeClosed == 17) {
            intakeSolenoid.set(Value.kForward);
        } else if (framesSinceIntakeClosed == 23) {
            intakeSolenoid.set(Value.kReverse);
        }

        // Close intake when ball stuck
        if (framesSinceIntakeOpen >= 50 && pdp.getCurrent(7) > 19 && conveyorReverseTimer < 0) {
            if (intakeOpen) {
                closeIntake(true);
            } else if (framesSinceIntakeClosed < 50) {
                closeIntake(true);
            }
        }

        // Stop warming up when finished warming up
        if (conveyorReverseTimer == 0) {
            shooterFalcon.set(ControlMode.Velocity, falconUnitsTargetVelocity);
            conveyor775.set(ControlMode.PercentOutput, 0);
            ballAtBarrel = false;
        }

        setLED();
        SmartDashboard.putBoolean("Warming", shooterWarming);
        framesSinceIntakeOpen++;
        framesSinceIntakeClosed++;
        conveyorReverseTimer--;
    }

}
