 
package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    
        SparkMax intake;
    
        public IntakeSubsystem(){
        intake = new SparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
    }  
        
    public void setIntakeMotors(double speed) {
        intake.set(speed);
    } 
    
    public void stop() {
        intake.set(0);
    }
}