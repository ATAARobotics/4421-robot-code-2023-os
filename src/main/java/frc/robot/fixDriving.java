package frc.robot;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;
public class fixDriving {
    double frontLeftAngle = SwerveDriveSubsystem.getModuleAngle(0);
    double frontRightAngle = SwerveDriveSubsystem.getModuleAngle(1);
    double rearLeftAngle = SwerveDriveSubsystem.getModuleAngle(2);
    double rearRightAngle = SwerveDriveSubsystem.getModuleAngle(3);
    double frontLeftTarget = SwerveDriveSubsystem.getModuleTargetAngle(0);
    double frontRightTarget = SwerveDriveSubsystem.getModuleTargetAngle(1);
    double rearLeftTarget = SwerveDriveSubsystem.getModuleTargetAngle(2);
    double rearRightTarget = SwerveDriveSubsystem.getModuleTargetAngle(3);
    double frontLeftOffset = frontLeftTarget - frontLeftAngle;
    double frontRightOffset = frontRightTarget - frontRightAngle;
    double rearLeftOffset = rearLeftTarget - rearLeftAngle;
    double rearRightOffset = rearRightTarget - rearRightAngle;
    
}
