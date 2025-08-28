package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class TurretSubsystem {
    private MotionMagicVoltage positionMain = 
        new MotionMagicVoltage(0); //Update MotionMagic constants
    private Follower positionFollower = 
        new Follower(0, false); 
    public TurretSubsystem(){
        
    }
//Stubbed functions until the turret subsystem is done.
public Rotation2d getGyroRotation2d(){
    return null;
}

public void zeroTurret(){
}

public boolean turretOnTarget(){
    return false;
}

public void goToPos(Pose2d target){
}
    
}
