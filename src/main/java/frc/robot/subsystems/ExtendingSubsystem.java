package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

     public class ExtendingSubsystem extends SubsystemBase {
        // The motors on the left side of the drive.
         
        CANSparkMax m_ArmExtend = new CANSparkMax(Constants.ArmConstants.ArmExtenderID, MotorType.kBrushless);

        private SparkMaxPIDController m_PIDControllerExtend;


        private RelativeEncoder m_encoderExtend;

        public double maxVel, maxAcc;

        double processVariable;


        

        public ExtendingSubsystem() {

         
            m_encoderExtend = m_ArmExtend.getEncoder();
//m_ArmExtend.setSoftLimit(63, 0)

            resetEncoders();

            m_ArmExtend.set(0);



            m_ArmExtend.restoreFactoryDefaults();



            m_ArmExtend.setInverted(true);

            m_ArmExtend.setIdleMode(IdleMode.kBrake);

            m_PIDControllerExtend = m_ArmExtend.getPIDController();


            m_PIDControllerExtend.setP(Constants.kArmExtendGains.kP);
            m_PIDControllerExtend.setI(Constants.kArmExtendGains.kI);
            m_PIDControllerExtend.setD(Constants.kArmExtendGains.kD);
            m_PIDControllerExtend.setFF(Constants.kArmExtendGains.kF);


         maxVel = 2838;
         maxAcc = 2838;

            int smartMotionSlot = 0;
            

            m_PIDControllerExtend.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
            m_PIDControllerExtend.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
            m_PIDControllerExtend.setSmartMotionAllowedClosedLoopError(.3, 0);


            
        }

        @Override
        public void periodic() {

        SmartDashboard.putNumber("ExtendPosition", m_encoderExtend.getPosition());


          // This method will be called once per scheduler run
        }


      

         public void ExtendOverride(double Speed){
        
            m_ArmExtend.set(Speed/2);

    
             }


        public void Extend(){
         //   resetEncoders();
            m_PIDControllerExtend.setReference(60, ControlType.kPosition, 0, 0, ArbFFUnits.kPercentOut);
           // m_PIDControllerExtend.setReference(joystickButton6, ControlType.kDutyCycle);
        }

        public void Retract(){
            m_PIDControllerExtend.setReference(0, ControlType.kPosition);
         //   m_PIDControllerExtend.setReference(joystickButton6, ControlType.kDutyCycle);
        }

        
        public void stop(){
            m_ArmExtend.set(0);
        }
        

         public void resetEncoders(){
             m_encoderExtend.setPosition(0);
        }

        public double getEncoderExtend(){

            return m_encoderExtend.getPosition();
        }
    }

 