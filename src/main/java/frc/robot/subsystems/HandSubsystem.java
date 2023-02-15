package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

     public class HandSubsystem extends SubsystemBase {
        // The motors on the left side of the drive.
         
        CANSparkMax m_HandMotor = new CANSparkMax(Constants.ArmConstants.HandMotorID, MotorType.kBrushless);
        
      
        private SparkMaxPIDController m_HandController;

        private RelativeEncoder m_HandEncoder;

        public HandSubsystem() {

            m_HandEncoder = m_HandMotor.getEncoder();            
   
            m_HandMotor.set(0);


            m_HandMotor.restoreFactoryDefaults();


            m_HandMotor.setInverted(false);

            

            m_HandMotor.setIdleMode(IdleMode.kBrake);


            m_HandController = m_HandMotor.getPIDController();

            m_HandController.setFeedbackDevice(m_HandEncoder);

            m_HandController.setP(.1);
            m_HandController.setI(0);
            m_HandController.setD(0);
            m_HandController.setFF(0);

            
        }

        @Override
        public void periodic() {
        }

        public void RunHands(){
        
            m_HandController.setReference(100, CANSparkMax.ControlType.kVelocity);
          //  processVariable = m_encoderActuate.getVelocity();
        }





    }

 