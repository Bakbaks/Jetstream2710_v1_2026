package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 10;
    public static final int kIntakeRollers = 11;
    public static final int kFloor = 12;
    public static final int kFeeder = 13;
    public static final int kHanger = 18;

    
    public static final int kFrontLeftShooter = 14;
    public static final int kBackLeftShooter = 15;
    
    public static final int kFrontRightShooter = 16;
    public static final int kBackRightShooter = 17;
    

}