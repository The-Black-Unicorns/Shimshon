package com.swervedrivespecialties.swervelib;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians, boolean matchEncoder);

    double getStateAngle();
}
