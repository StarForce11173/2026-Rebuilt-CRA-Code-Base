 
package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    
        SparkMax LeftIntake;
        SparkMax RightIntake;
    
        public IntakeSubsystem(){
        LeftIntake = new SparkMax(IntakeConstants.kLeftIntakeMotorCanId, MotorType.kBrushless);
        RightIntake = new SparkMax(IntakeConstants.kRightIntakeMotorCanId, MotorType.kBrushless);
//67
    }  
        
    public void setIntakeMotors(double speed) {
        LeftIntake.set(speed);
        RightIntake.set(speed);
    } 
    
    public void stop() {
        LeftIntake.set(0);
        RightIntake.set(0);
    }
}