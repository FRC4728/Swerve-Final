package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsytem {}
 /*    public class DriveSubsystem extends SubsystemBase {
        // The motors on the left side of the drive.
         
        CANSparkMax m_ArmMaster = new CANSparkMax(61, MotorType.kBrushless);
        CANSparkMax m_ArmExtend = new CANSparkMax(60, MotorType.kBrushless);
        
        private final DoubleSolenoid m_doubleSolenoid = 
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
      
        private SparkMaxPIDController m_PIDControllerActuate;
        private SparkMaxPIDController m_PIDControllerExtend;

        private RelativeEncoder m_encoderActuate;
        private RelativeEncoder m_encoderExtend;

        public double maxVel, maxAcc;

        double setPoint, setPoint2, processVariable;
        //double check this works
        public void setSetPoint(double setPoint) {
            this.setPoint = 0;
        }

        public void setSetPoint2(double setPoint2) {
            this.setPoint = 90;
        }

        

        public void ArmSubsystem() {

            resetEncoders ();

            m_encoderActuate = m_ArmMaster.getEncoder();
            m_encoderExtend = m_ArmExtend.getEncoder();

            m_ArmMaster.set(0);
            m_ArmExtend.set(0);

            m_ArmMaster.restoreFactoryDefaults();
            m_ArmExtend.restoreFactoryDefaults();

            m_ArmMaster.setInverted(false);
            m_ArmExtend.setInverted(false);

            m_ArmMaster.setIdleMode(IdleMode.kBrake);
            m_ArmExtend.setIdleMode(IdleMode.kBrake);

            m_PIDControllerActuate = m_ArmMaster.getPIDController();
            m_PIDControllerExtend = m_ArmExtend.getPIDController();

            m_PIDControllerActuate.setP(Constants.gains.kArmGains.kP);
            m_PIDControllerActuate.setI(Constants.gains.kArmGains.kI);
            m_PIDControllerActuate.setD(Constants.gains.kArmGains.kD);
            m_PIDControllerActuate.setFF(Constants.gains.kArmGains.kF);

            m_PIDControllerExtend.setP(Constants.gains.kArmExtend.kP);
            m_PIDControllerExtend.setI(Constants.gains.kArmExtend.kI);
            m_PIDControllerExtend.setD(Constants.gains.kArmExtend.kD);
            m_PIDControllerExtend.setFF(Constants.gains.kArmExtend.kF);

            maxVel = 5676;
            maxAcc = 5676;

            int smartMotionSlot = 0;
            m_PIDControllerActuate.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
            m_PIDControllerActuate.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);

           
            
        }

        public void ActuateUp(double joystickButton3){
        
            m_PIDControllerActuate.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
            processVariable = m_encoderActuate.getVelocity();
        }

        public void ActuateRest(double joystickButton4){
        
            m_PIDControllerActuate.setReference(setPoint2, CANSparkMax.ControlType.kVelocity);
            processVariable = m_encoderActuate.getVelocity();
        }

        public void Extend(double joystickButton5, double joystickButton6){
            m_PIDControllerExtend.setReference(joystickButton5, ControlType.kDutyCycle);
            m_PIDControllerExtend.setReference(joystickButton6, ControlType.kDutyCycle);
        }

         public void resetEncoders(){
             m_encoderActuate.setPosition(0);
             m_encoderExtend.setPosition(0);
        }

        public double getEncoderActuate(){

            return m_encoderActuate.getPosition();
        }

        public double getEncoderExtend(){

            return m_encoderExtend.getPosition();
        }
    }
}
 */