package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HopperSim extends Hopper {
  ShooterSim shooter;
  int time = 0;
  static int ballsIn = 8;
  boolean kicking = false;

  public HopperSim(ShooterSim shooter) {
    super();
    this.shooter = shooter;
  }

  @Override
  public Command shootCMD() {
    return runOnce(
            () -> {
              kicking = true;
            })
        .andThen(
            Commands.repeatingSequence(
                    Commands.waitUntil(
                        () -> {
                          return ballsIn > 0;
                        }),
                    Commands.waitSeconds(0.3),
                    runOnce(
                        () -> {
                          shooter.shoot();
                          ballsIn--;
                        }))
                .until(
                    () -> {
                      return !kicking;
                    }));
  }

  @Override
  public Command stopCMD() {
    return runOnce(
        () -> {
          kicking = false;
        });
  }

  public static void intake() {
    ballsIn++;
  }
}
