package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final CANSparkMax driveMotor, turnMotor;
    private final RelativeEncoder driveEncoder, turnEncoder;
    private final PIDController turnPIDController;
    private final CANcoder absoluteEncoder;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean isAbsoluteEncoderReversed){
        //CANCoder Configuration
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        config.MagnetSensor.SensorDirection = isAbsoluteEncoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        
        //Initializing Motors & Motor Config
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        turnMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveMotor.setSmartCurrentLimit(30);
        turnMotor.setSmartCurrentLimit(20);

        //Initializing Encoders & Encoder Config
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
        driveEncoder = driveMotor.getEncoder(Type.kHallSensor, 42);
        turnEncoder = turnMotor.getEncoder(Type.kHallSensor, 42);

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRPM2RadPerSec);

        //Initializing PIDs & PID Config
        turnPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        //Reset Encoder Position Upon RobotInit
        resetEncoders();
    }

    /* Get Functions */

    //Position
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }

    //Velocity
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }

    //Rotation
    public double getAbsoluteEncoderRad() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI;
    }

    //Module State & Position
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getTurningPosition()));
    }

    /* Reset Functions */
    
    public void resetTurn(){
        double position = getAbsoluteEncoderRad();
        turnEncoder.setPosition(position);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        resetTurn();
    }

    /* Set Functions */

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void setDesiredState(SwerveModuleState state) {
        //Stop the module if it's moving less than 0.001 m/s.
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        //Apply angle optimization to the state
        state = SwerveModuleState.optimize(state, getState().angle);

        //Set speeds for the drive and turn motors
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(turnPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
    }
}
