package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");
    

    // Talon FX IDs
    public static final int kIntakePivot = 31;
    public static final int kIntakeRollers = 32;
    public static final int kHopperMotor = 40;
    public static final int kFeederMotor = 41;

}
