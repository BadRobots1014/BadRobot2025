package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurnThetaHelper {
    double driveThetad;

    double currentTheta;
    double targetTheta;
    // Supplier<Double> targetTheta;

    PIDController turningPID;

    // driveTheta is the input for turning (turnSpeed)
    public Supplier<Double> driveTheta = new Supplier<Double>() {
        @Override
        public Double get() {
            return driveThetad;
        }
    };

    public TurnThetaHelper(double currentTheta) {
        this.currentTheta = currentTheta;

        turningPID = new PIDController(1, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // // Sets desired angle in radians
    // public void setTargetTheta(Supplier<Double> targetTheta) {
    //     this.targetTheta = targetTheta;
    // }

    // Place in execute function of command to keep feeding PID
    // Requires subsystem.getRotation2d() to be passed
    public void calculate(Rotation2d currentTheta, Rotation2d targetTheta) {
        this.currentTheta = currentTheta.getRadians();
        this.targetTheta = targetTheta.getRadians();

        driveThetad = turningPID.calculate(this.currentTheta, this.targetTheta);
    }
}
