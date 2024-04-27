package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

    //Initializing Modules & Gyroscope
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRot,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurningMotorPort, 
        DriveConstants.kFrontRightDriveReversed, 
        DriveConstants.kFrontRightTurningEncoderReversed, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRot, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveReversed, 
        DriveConstants.kBackLeftTurningEncoderReversed, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRot, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightTurningMotorPort, 
        DriveConstants.kBackRightDriveReversed,
        DriveConstants.kBackRightTurningEncoderReversed, 
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRot, 
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private AHRS gyro = new AHRS(SPI.Port.kMXP);


    public SwerveSubsystem() {
        //Reset the gyroscope and drive encoders (wait for CANcoders to return normal values)
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetGyro();
                resetDriveEncoders();
            } catch (Exception e) {}
        }).start();
    }

    /* Get Functions */

    public double getHeading(){
        //Changes the -180->0 to 180->360 (0 to 180 stays the same)
        double heading = Math.IEEEremainder(-gyro.getYaw(), 360);

        if(heading < 0){
            heading = 180 + (180+heading);
        }

        return heading;
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(), 
            frontRight.getState(), 
            backLeft.getState(),
            backRight.getState()
        );
    }

    /* Reset Functions */
    public void resetDriveEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void resetTurn(){
        frontLeft.resetTurn();
        frontRight.resetTurn();
        backLeft.resetTurn();
        backRight.resetTurn();
    }

    public void resetGyro(){
        gyro.reset();
    }

    /* Set Functions */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

    public void setModuleStatesFromSpeeds(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}
