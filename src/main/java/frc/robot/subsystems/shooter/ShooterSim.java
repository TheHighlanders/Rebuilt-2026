package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
// to implement
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FuelSim;

public class ShooterSim extends Shooter {
  private FuelSim fuelSim;

  SparkMaxSim flywheelSim = new SparkMaxSim(flywheel, DCMotor.getKrakenX60(1));

  public ShooterSim(FuelSim fuelSim) {
    super();
    this.fuelSim = fuelSim;
  }

  @Override
  public boolean atSpeed() {
    return flywheelSim.getVelocity() >= targetRPM - 10; // tolerance, trust
  }

  public Command shootCMD() {
    return runOnce(
        () -> {
          fuelSim.launchFuel(
              MetersPerSecond.of(
                  (flywheelSim.getVelocity() / 60)
                      * ShooterConstants.FLYWHEEL_RADIUS.in(Meters)
                      * 2
                      * Math.PI),
              ShooterConstants.SHOOTER_HOOD,
              Degrees.of(-90),
              ShooterConstants.SHOOTER_RR_POS);
        });
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
  }
}
