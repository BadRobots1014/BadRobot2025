package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.subsystems.SwerveSubsystem;

public class SnapToThetaCommand extends TurnToThetaCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  Supplier<Double> unsnappedTargetTheta;
  double increment;

  public SnapToThetaCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> targetTheta, Supplier<Double> moveX,
      Supplier<Double> moveY, boolean fieldOriented, Supplier<Boolean> angleRelevant, double incrementDegrees) {
    super(
        swerveSubsystem,
        () -> 0d,
        moveX,
        moveY,
        fieldOriented,
        angleRelevant);
    this.unsnappedTargetTheta = targetTheta;
    this.increment = incrementDegrees / 180 * Math.PI;
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {

    super.targetTheta = new Supplier<Double>() {
      @Override
      public Double get() {
        var snappedTheta = unsnappedTargetTheta.get() - Math.IEEEremainder(unsnappedTargetTheta.get(), increment);
        System.out.println("Unsnapped theta: " + unsnappedTargetTheta.get() * 180 / Math.PI + "\nSnapped theta: " + snappedTheta  * 180 / Math.PI);
        return snappedTheta;
      }
    };

    super.execute();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}