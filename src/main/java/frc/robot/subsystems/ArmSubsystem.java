package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

     public class ArmSubsystem extends SubsystemBase {
        // The motors on the left side of the drive.
         
        CANSparkMax m_ArmMaster = new CANSparkMax(61, MotorType.kBrushless);
        CANSparkMax m_ArmFollower = new CANSparkMax(62, MotorType.kBrushless);
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

        

        public ArmSubsystem() {

            resetEncoders ();

            m_encoderActuate = m_ArmMaster.getEncoder(Type.kQuadrature, 406);
            m_encoderExtend = m_ArmExtend.getEncoder();
            

            m_ArmMaster.set(0);
            m_ArmExtend.set(0);
            m_ArmFollower.set(0);

            m_ArmMaster.restoreFactoryDefaults();
            m_ArmExtend.restoreFactoryDefaults();
            m_ArmFollower.restoreFactoryDefaults();

            m_ArmMaster.setInverted(false);
            m_ArmExtend.setInverted(false);
            

            m_ArmMaster.setIdleMode(IdleMode.kBrake);
            m_ArmExtend.setIdleMode(IdleMode.kBrake);

            m_PIDControllerActuate = m_ArmMaster.getPIDController();
            m_PIDControllerExtend = m_ArmExtend.getPIDController();

            m_PIDControllerActuate.setP(Constants.kArmGains.kP);
            m_PIDControllerActuate.setI(Constants.kArmGains.kI);
            m_PIDControllerActuate.setD(Constants.kArmGains.kD);
            m_PIDControllerActuate.setFF(Constants.kArmGains.kF);

            m_PIDControllerExtend.setP(Constants.kArmExtendGains.kP);
            m_PIDControllerExtend.setI(Constants.kArmExtendGains.kI);
            m_PIDControllerExtend.setD(Constants.kArmExtendGains.kD);
            m_PIDControllerExtend.setFF(Constants.kArmExtendGains.kF);
            
            maxVel = 5676;
            maxAcc = 5676;

            int smartMotionSlot = 0;
            m_PIDControllerActuate.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
            m_PIDControllerActuate.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);

            m_ArmFollower.follow(m_ArmMaster);
            
        }

        public void ActuateUp(){
        
            m_PIDControllerActuate.setReference(90, CANSparkMax.ControlType.kSmartMotion);
           // processVariable = m_encoderActuate.getVelocity();
        }

        public void ActuateRest(){
        
            m_PIDControllerActuate.setReference(0, CANSparkMax.ControlType.kSmartMotion);
          //  processVariable = m_encoderActuate.getVelocity();
        }

        public void Extend(){
            m_PIDControllerExtend.setReference(1, ControlType.kPosition);
         //   m_PIDControllerExtend.setReference(joystickButton6, ControlType.kDutyCycle);
        }

        public void Retract(){
            m_PIDControllerExtend.setReference(0, ControlType.kPosition);
         //   m_PIDControllerExtend.setReference(joystickButton6, ControlType.kDutyCycle);
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

 