package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kFloorL = 30;
    public static final int kFloorR = 62; 
    public static final int kFeederL = 52; 
    public static final int kFeederR = 51; 

    public static final int kBottomRightShooter = 15; 
    public static final int kBottomLeftShooter = 14; 
    
    public static final int kTopRightShooter = 17; 
    public static final int kTopLeftShooter = 16; 

    public static final int kIntakeExtendo = 12;
    public static final int kIntakeRollers = 13;
    public static final int kIntakeRollers2 = 20;
    

}