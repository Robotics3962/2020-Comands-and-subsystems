package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Util {

    public static void configTalon(WPI_TalonSRX talon){
        //Tells the talon that the max output that it can give is between 1 and -1 which would mean full forward and full backward.
        //There is no allowance currently for anything in between
        talon.configPeakOutputForward(1,0);
        talon.configPeakOutputReverse(-1,0);

        //Tells the talon that it should current limit itself so that we don't blow a 40amp breaker.
        talon.configPeakCurrentLimit(40,0);
        talon.enableCurrentLimit(true);
        talon.configContinuousCurrentLimit(40,0);
        //The max output current is 40Amps for .25 of a second
        talon.configPeakCurrentDuration(250, 0);

        //Tells the talon that is should only appy 12 volts or less to the motor.
        talon.configVoltageCompSaturation(12,0);
    } 

    public static void configTalonSRX(TalonSRX talon){
        //Tells the talon that the max output that it can give is between 1 and -1 which would mean full forward and full backward.
        //There is no allowance currently for anything in between
        talon.configPeakOutputForward(1,0);
        talon.configPeakOutputReverse(-1,0);

        //Tells the talon that it should current limit itself so that we don't blow a 40amp breaker.
        talon.configPeakCurrentLimit(40,0);
        talon.enableCurrentLimit(true);
        talon.configContinuousCurrentLimit(40,0);
        //The max output current is 40Amps for .25 of a second
        talon.configPeakCurrentDuration(250, 0);

        //Tells the talon that is should only appy 12 volts or less to the motor.
        talon.configVoltageCompSaturation(12,0);
    } 
}
