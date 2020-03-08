package teamcode.roadrunner.hardware;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.ModernRoboticsMotorControllerParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev=537.6, gearing=19.2, maxRPM=340, orientation= Rotation.CW)
@DeviceProperties(xmlTag="NeveRest20OrbitalGearmotor", name="NeveRest Orbital 20 Gearmotor", builtIn = true)
@DistributorInfo(distributor="AndyMark", model="am-3102", url="http://www.andymark.com/NeveRest-20-12V-Gearmotor-p/am-3102.htm")
@ModernRoboticsMotorControllerParams(P=160, I=32, D=112, ratio=25)
public interface NeveRestOrbital20Gearmotor
{
}
