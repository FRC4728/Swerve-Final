package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    
    private final DoubleSolenoid m_doubleSolenoid = 
    new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  public HopperSubsystem() {

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
