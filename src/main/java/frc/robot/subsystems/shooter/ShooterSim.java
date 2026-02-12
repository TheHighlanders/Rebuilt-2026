package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
// to implement
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FuelSim;

public class ShooterSim extends Shooter {
  private FuelSim fuelSim;

  private int ballsIn = 0;

  private int time = 0;

  private boolean kicking = false;

  SparkMaxSim flywheelSim = new SparkMaxSim(flywheel, DCMotor.getKrakenX60(1));

  public ShooterSim(FuelSim fuelSim) {
    super();
    this.fuelSim = fuelSim;
  }

  @Override
  public Command kickerCMD() {
    return runOnce(
        () -> {
          kicking = true;
        });
  }

  @Override
  public Command stopCMD() {
    return runOnce(
        () -> {
          kicking = false;
          targetRPM = 0;
        });
  }

  @Override
  public boolean atSpeed() {
    return flywheelSim.getVelocity() >= targetRPM - 10; // tolerance, trust
  }

  public void intake() {
    ballsIn++;
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)

    // Next, we update it. The standard loop time is 20ms.

    // Now, we update the Spark MAX
    flywheelSim.iterate(
        targetRPM, 12, // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds
    SmartDashboard.putNumber("shootersim/flywheelspd", flywheelSim.getVelocity());
    SmartDashboard.putNumber("shootersim/flywheelsetpoint", targetRPM);

    if (kicking) {
      if (time >= 17 && ballsIn > 0) {
        time = 0;
        ballsIn--;
        fuelSim.launchFuel(
            InchesPerSecond.of((flywheelSim.getVelocity() / 60) * Math.PI * 4),
            Degrees.of(78),
            Degrees.of(-90),
            ShooterConstants.SHOOTER_RR_POS);
      }
    }
    if (ballsIn != 0) time++;
    SmartDashboard.putBoolean("shootersim/kicking?", kicking);
    SmartDashboard.putNumber("shootersim/balls in", ballsIn);
  }
}
