package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;
import java.util.Queue;

public class GyroIOBoron implements GyroIO {
  private final Canandgyro gyro = new Canandgyro(DriveConstants.GYRO_ID);
  public double yaw = gyro.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOBoron() {
    gyro.getPitch();
    gyro.getRoll();
    CanandgyroSettings settings = new CanandgyroSettings();
    settings.setYawFramePeriod(1 / Drive.ODOMETRY_FREQUENCY);
    settings.setAngularVelocityFramePeriod(1 / Drive.ODOMETRY_FREQUENCY);
    CanandgyroSettings gyroConfigStatus = gyro.setSettings(settings, 0.02, 5);

    if (!gyroConfigStatus.isEmpty()) {
      DriverStation.reportError("Gyro failed to configure", false);
    }
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(() -> gyro.getMultiturnYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // check connection with 60ms timeout instead of 2s default due to frequency of gyro reporting
    // failure
    inputs.connected = gyro.isConnected(0.06);
    inputs.yawPosition = Rotation2d.fromRotations(gyro.getMultiturnYaw());
    inputs.yawVelocityRadPerSec = Units.rotationsToRadians(gyro.getAngularVelocityYaw());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  public Canandgyro getGyro() {
    return gyro;
  }
}
