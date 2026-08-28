package frc.robot;

public final class Constants {

    public static final int TITAN_ID = 42;

    public static final int MOTOR_0 = 0; //front
    public static final int MOTOR_2 = 2; //left
    public static final int MOTOR_3 = 3; //right
    public static final int MOTOR_1 = 1;

    public static final int ENCODER_0 = 0;
    public static final int ENCODER_2 = 2;
    public static final int ENCODER_3 = 3;
    public static final int ENCODER_1 = 1;

    public static final int BTN_START     = 10;
    public static final int BTN_STOP      = 9;
    public static final int LEDRun        = 13; // led rodando
    public static final int LEDStop       = 14; // led parando
    

    public static final int TRIG1         = 12;
    public static final int ECHO1         = 11;
    public static final int ULTRA1        = 0;
    public static final int ULTRA2        = 1;

    public static final double VELOCIDADE = 0.5;
    public static final double DIST_PER_TICK = 0.000214659;

    public static final double wheelRadius = 100; //mm
    public static final double pulsePERRevolution = 1440;
    public static final double gearRatio = 1/1;
    public static final double wheelPulseRatio = pulsePERRevolution * gearRatio;
    public static final double WHEEL_DIST_PER_TICK = (Math.PI * 2 * wheelRadius) / wheelPulseRatio;
}