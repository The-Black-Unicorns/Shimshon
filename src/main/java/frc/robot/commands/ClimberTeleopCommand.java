// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTeleopCommand extends CommandBase {

    ClimberSubsystem climberSubsystem;
    PS4Controller secondController;
    Joystick mainController;

    boolean keepOutsideClosed = true;

    public ClimberTeleopCommand(ClimberSubsystem subsystem, Joystick taranis, PS4Controller controller) {
        climberSubsystem = subsystem;
        mainController = taranis;
        secondController = controller;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climberSubsystem.setLimitsEnabled(!mainController.getRawButton(6));

        // climberSubsystem.setBrake(secondController.getSquareButton());

        boolean resetInsideArm = secondController.getShareButton();
        if (!resetInsideArm) {
            climberSubsystem.moveInsideArm(-deadband(secondController.getLeftY(), 0.2));
        }
        if (secondController.getShareButtonPressed())
            climberSubsystem.startResetInsideArmsLength();
        if (secondController.getShareButtonReleased())
            climberSubsystem.stopResetInsideArm(false);

        boolean resetOutsideArm = secondController.getOptionsButton();
        if (!resetOutsideArm && !secondController.getPSButton()) {

            if (mainController.getRawButton(5)) {
                climberSubsystem.moveOutsideArm(deadband(mainController.getRawAxis(2), 0.2));
            } else if (keepOutsideClosed) {
                climberSubsystem.checkForExtensionOutside();
            }
        }
        if (secondController.getOptionsButtonPressed())
            climberSubsystem.startResetOutsideArmsLength();
        if (secondController.getOptionsButtonReleased())
            climberSubsystem.stopResetOutsideArm(false);

        if (secondController.getPSButtonPressed()) {
            keepOutsideClosed = false;
            climberSubsystem.startOutsideOpen();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private double deadband(double value, double lowerLimit) {
        // return Math.abs(value) < lowerLimit ? 0 : value;
        if (Math.abs(value) < lowerLimit) {
            return 0;
        } else {
            if (value > 0) {
                return (value - lowerLimit) / (1 - lowerLimit);
            } else {
                return (value + lowerLimit) / (1 - lowerLimit);
            }
        }
    }
}
