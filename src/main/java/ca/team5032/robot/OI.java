package ca.team5032.robot;

public class OI {

    // Falcon 500 stuff
    public static final int UNITS_PER_ROTATION = 2048;
    public static final double WHEEL_CIRCUMFERENCE = 6 * Math.PI;

    // Vision Constants
    public static final double LIMELIGHT_HEIGHT_OF_LIMELIGHT = 31.67 / 39.37; // in m
    public static final double LIMELIGHT_HEIGHT_OF_INNER_PORT = 98.25 /  39.37; // in m
    public static final double LIMELIGHT_ANGLE = 26.03; // in degrees
    public static final double LIMELIGHT_HEIGHT_OF_SHOOTER = 31.66 / 39.37; // in m
    public static final double LIMELIGHT_SHOOTER_ANGLE = 30; // in degrees
    public static final double GRAVITY = 9.80665; // Gravity in m/s^2

    // Shooter Constants
    public static final int TIMEOUT = 30; // in ms
    public static final double MAG_UNITS_PER_ROTATION = 4096.0; // Ticks per rotation for encoder
    public static final double LAUNCHER_RADIUS = 2 / 39.37; // in m
    public static final double MAG_SPEED_TIME = 10; // Sensor outputs in ticks per 100 ms we need ticks per 1000 ms or 1 s so multiply by 10
    public static final boolean DISCONTINUITY_PRESENT = true;
	public static final int BOOK_END_0 = 910;   /* 80 deg */
    public static final int BOOK_END_1 = 1137;	/* 100 deg */
    public static final double MAX_SPEED = (10000 / 60 * 2 * Math.PI) * LAUNCHER_RADIUS; // In m/s incorporates friction and stuff
    public static final double MIN_SPEED = (2000 / 60 * 2 * Math.PI) * LAUNCHER_RADIUS; // In m/s incorporates friction and stuff
    public static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.000620396; // in kg * m ^ 2 calculated using physics for state-space math PID change for physical robot
    public static final double FLYWHEEL_GEARING = 1.0; // gear ratio for state-space math PID change for physical robot

    // TODO: actual values
    // TODO: better names
    public static final int SYSTEM_CAMERA_USBCAMERA_1 = 0;
    public static final int SYSTEM_CAMERA_USBCAMERA_2 = 1;

    public static final int SYSTEM_CLIMB_TALON_LEFT_ID = 7;
    public static final int SYSTEM_CLIMB_TALON_RIGHT_ID = 8;
    // public static final int SYSTEM_CLIMB_HOOKER_ID = 9;

    public static final int SYSTEM_INTAKE_VICTOR_INTAKE_ID = 3;
    public static final int SYSTEM_INTAKE_VICTOR_ROTATE_ID = 10;

    public static final int SYSTEM_SHOOTER_SPEED_VICTOR_ID = 6;
    public static final int SYSTEM_SHOOTER_HOOD_VICTOR_ID = 4;

    public static final int SYSTEM_TRANSFER_CHANNEL_BOTTOM_ID = 2;

    public static final int DRIVE_JOYSTICK_ID = 0;
    public static final int COSMETIC_JOYSTICK_ID = 1;

    public static final String SYSTEM_PREFIX = "System";

    public static final int COMMAND_ALIGN_SLOWING_RADIUS = 16;
    public static final double COMMAND_ALIGN_MAX_VELOCITY = 0.5;
    public static final double COMMAND_ALIGN_MIN_VELOCITY = 0.4;
}
