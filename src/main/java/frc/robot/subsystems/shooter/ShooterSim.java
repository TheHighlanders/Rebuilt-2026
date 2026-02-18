package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// to implement
import frc.robot.Constants.ShooterConstants;
import frc.robot.FuelSim;

public class ShooterSim extends Shooter {
  private FuelSim fuelSim;

  private TalonFXSimState flywheelSimState = flywheel.getSimState();
  private DCMotorSim flywheelSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.01, 1),
          DCMotor.getKrakenX60Foc(1));

  public ShooterSim(FuelSim fuelSim) {
    super();
    this.fuelSim = fuelSim;
    flywheelSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  public void shoot() {
    fuelSim.launchFuel(
        MetersPerSecond.of(
            flywheel.getVelocity().getValueAsDouble()
                * ShooterConstants.FLYWHEEL_RADIUS.in(Meters)
                * 2
                * Math.PI),
        ShooterConstants.SHOOTER_HOOD,
        Degrees.of(-90),
        ShooterConstants.SHOOTER_RR_POS);
  }

  @Override
  public void simulationPeriodic() {
    flywheelSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    flywheelSim.setInputVoltage(flywheelSimState.getMotorVoltage());
    flywheelSim.update(0.02);

    // flywheelSimState.setRawRotorPosition(flywheelSim.getAngularPosition());
    flywheelSimState.setRotorVelocity(targetRPS); // not true but works for now
  }
}
