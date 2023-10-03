package frc.robot;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;
public class fixDriving {
    static double frontLeftAngle = SwerveDriveSubsystem.getModuleAngle(0);
    static double frontRightAngle = SwerveDriveSubsystem.getModuleAngle(1);
    static double rearLeftAngle = SwerveDriveSubsystem.getModuleAngle(2);
    static double rearRightAngle = SwerveDriveSubsystem.getModuleAngle(3);
    static double frontLeftTarget = SwerveDriveSubsystem.getModuleTargetAngle(0);
    static double frontRightTarget = SwerveDriveSubsystem.getModuleTargetAngle(1);
    static double rearLeftTarget = SwerveDriveSubsystem.getModuleTargetAngle(2);
    static double rearRightTarget = SwerveDriveSubsystem.getModuleTargetAngle(3);
    static double frontLeftOffset = frontLeftTarget - frontLeftAngle;
    static double frontRightOffset = frontRightTarget - frontRightAngle;
    static double rearLeftOffset = rearLeftTarget - rearLeftAngle;
    static double rearRightOffset = rearRightTarget - rearRightAngle;
    static double [] offsets = {frontLeftOffset, frontRightOffset, rearLeftOffset, rearRightOffset};
    public static double getOffsetDifference(int module) {
        return offsets[module];
    }
}