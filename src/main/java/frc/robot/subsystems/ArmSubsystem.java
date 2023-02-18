package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    // The motors on the left side of the drive.

    CANSparkMax m_ArmMaster = new CANSparkMax(Constants.ArmConstants.ArmMasterID, MotorType.kBrushless);
    CANSparkMax m_ArmFollower = new CANSparkMax(Constants.ArmConstants.ArmFollowerID, MotorType.kBrushless);

    private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            Constants.ArmConstants.ArmPCMForward, Constants.ArmConstants.ArmPCMBackwards);

    private SparkMaxPIDController m_PIDControllerActuate;

    private RelativeEncoder m_encoderActuate;

    private DutyCycleEncoder angleEncoder;

    public double maxVel, maxAcc;

    double processVariable;

    public ArmSubsystem() {

        angleEncoder = new DutyCycleEncoder(new DigitalInput(Constants.ArmConstants.ArmAbsoluteActuator));
        m_encoderActuate = m_ArmMaster.getAlternateEncoder(AlternateEncoderType.kQuadrature, 4096);
        // m_encoderActuate = m_ArmMaster.getEncoder();

        resetEncoders();

        new Thread(() -> {
            try {
                Thread.sleep(1500);
                resetToAbsolute();

            } catch (Exception e) {
            }
        }).start();

        m_ArmMaster.set(0);
        m_ArmFollower.set(0);

        m_ArmMaster.restoreFactoryDefaults();
        m_ArmFollower.restoreFactoryDefaults();

        m_ArmMaster.setInverted(true);
        m_ArmFollower.setInverted(true);

        m_encoderActuate.setPositionConversionFactor(386.909091);

        m_ArmMaster.setIdleMode(IdleMode.kBrake);
        m_ArmFollower.setIdleMode(IdleMode.kBrake);

        m_PIDControllerActuate = m_ArmMaster.getPIDController();

        m_PIDControllerActuate.setFeedbackDevice(m_encoderActuate);

        // fix
        // m_encoderActuate.setPositionConversionFactor(maxAcc);

        m_PIDControllerActuate.setP(Constants.kArmGains.kP, 0);
        m_PIDControllerActuate.setI(Constants.kArmGains.kI, 0);
        m_PIDControllerActuate.setD(Constants.kArmGains.kD, 0);
        m_PIDControllerActuate.setFF(Constants.kArmGains.kF, 0);

        m_PIDControllerActuate.setP(Constants.kArmGains1.kP, 1);
        m_PIDControllerActuate.setI(Constants.kArmGains1.kI, 1);
        m_PIDControllerActuate.setD(Constants.kArmGains1.kD, 1);
        m_PIDControllerActuate.setFF(Constants.kArmGains1.kF, 1);

        
        m_PIDControllerActuate.setP(Constants.kArmGains1.kP, 2);
        m_PIDControllerActuate.setI(Constants.kArmGains1.kI, 2);
        m_PIDControllerActuate.setD(Constants.kArmGains1.kD, 2);
        m_PIDControllerActuate.setFF(Constants.kArmGains1.kF, 2);

        // maxVel = 5676;
        // maxAcc = 5676;

        maxVel = 3500;
        maxAcc = 3500;

        int smartMotionSlot = 0;

        m_ArmMaster.enableVoltageCompensation(12);
        m_PIDControllerActuate.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_PIDControllerActuate.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_PIDControllerActuate.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

        m_PIDControllerActuate.setSmartMotionMaxVelocity(maxVel, 1);
        m_PIDControllerActuate.setSmartMotionMaxAccel(maxAcc, 1);
        m_PIDControllerActuate.setSmartMotionAllowedClosedLoopError(.1, 1);

        m_ArmFollower.follow(m_ArmMaster, false);

        m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Absolute Position", angleEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("ArmPosition", m_encoderActuate.getPosition());

    }

    public void ActuateUp() {

        m_PIDControllerActuate.setReference(110, CANSparkMax.ControlType.kSmartMotion, 0, .12, ArbFFUnits.kPercentOut);
        processVariable = m_encoderActuate.getPosition();
        // m_PIDControllerActuate.setReference(.05, CANSparkMax.ControlType.kPosition);
        // processVariable = m_encoderActuate.getPosition();
    }

    public void Actuate(double Speed) {

        m_ArmFollower.set(Speed / 4);
        m_ArmMaster.set(Speed / 4);
    }

    public void ActuateToHopper() {

        m_PIDControllerActuate.setReference(-18.8, CANSparkMax.ControlType.kSmartMotion, 1, -.12,
                ArbFFUnits.kPercentOut);
        processVariable = m_encoderActuate.getPosition();
        
    }

    
    public void ActuateGround() {

        m_PIDControllerActuate.setReference(32, CANSparkMax.ControlType.kSmartMotion, 1, -.12, ArbFFUnits.kPercentOut);
        processVariable = m_encoderActuate.getPosition();

    }

    public void ActuateHome() {

        m_PIDControllerActuate.setReference(0, CANSparkMax.ControlType.kSmartMotion, 1, 0, ArbFFUnits.kPercentOut);
        processVariable = m_encoderActuate.getPosition();

    }
    public void ActuateMiddle() {
        m_PIDControllerActuate.setReference(90, CANSparkMax.ControlType.kSmartMotion, 0, .12, ArbFFUnits.kPercentOut);

    }
    public void PneumaticsToggle() {
        m_doubleSolenoid.toggle();
    }

    public void PistonArmIn() {
        m_doubleSolenoid.set(Value.kReverse);
    }

    public void PistonArmOut() {
        m_doubleSolenoid.set(Value.kForward);
    }

    public void Stop(){
        m_PIDControllerActuate.setReference(0, ControlType.kPosition);
    }
    public Value PistonArmExtended() {
       return m_doubleSolenoid.get();
    }

    public void resetEncoders() {
        m_encoderActuate.setPosition(0);
    }

    public void resetToAbsolute() {
        double absolutePosition = (angleEncoder.getAbsolutePosition() * 386.909091)
                - Constants.ArmConstants.AbsoluteArmOffset;
        m_encoderActuate.setPosition(absolutePosition);
    }

    public double getEncoderActuate() {

        return m_encoderActuate.getPosition();
    }

}
