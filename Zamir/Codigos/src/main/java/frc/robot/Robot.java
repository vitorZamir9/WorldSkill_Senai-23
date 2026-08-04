package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.studica.frc.Titan;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    // =========================================================
    //  Configuracoes de movimento e giro
    // =========================================================
    private static final double VEL_MOVE        = 0.7;
    private static final double VEL_GIRO        = 0.35;
    private static final double TOLERANCE_DEG   = 3.0;
    private static final int    CICLOS_ESTAVEIS = 10;

    // =========================================================
    //  Hardware
    // =========================================================
    private AHRS navx;
    private Titan titan;
    private Titan.Motor motor0, motor2, motor3;
    private DigitalInput  btnStart, btnStop;
    private DigitalOutput led1, led2;
    private boolean lastStart = false, lastStop = false;

    // =========================================================
    //  Estado do giro
    // =========================================================
    private boolean giroIniciado   = false;
    private int     ciclosEstaveis = 0;
    private double  anguloBase     = 0.0; // angulo do navx (getAngle) no inicio do giro
    private double  giroAlvo       = 0.0; // quantidade de graus a girar a partir de anguloBase

    // =========================================================
    @Override
    public void robotInit() {
        navx = new AHRS(SPI.Port.kMXP);

        titan  = new Titan(Constants.TITAN_ID);
        motor0 = titan.getMotor(Constants.MOTOR_0);
        motor2 = titan.getMotor(Constants.MOTOR_2);
        motor3 = titan.getMotor(Constants.MOTOR_3);

        btnStart = new DigitalInput(Constants.BTN_START);
        btnStop  = new DigitalInput(Constants.BTN_STOP);
        led1     = new DigitalOutput(Constants.LEDRun);
        led2     = new DigitalOutput(Constants.LEDStop);
    }

    @Override
    public void robotPeriodic() {
        final boolean curStart = btnStart.get();
        final boolean curStop  = btnStop.get();
        if (lastStart && !curStart) { led1.set(true);  led2.set(false); }
        if (lastStop  && !curStop)  { led1.set(false); led2.set(true);  }
        lastStart = curStart;
        lastStop  = curStop;

        SmartDashboard.putNumber("navx/angle", navx.getAngle());
    }

    // =========================================================
    @Override
    public void autonomousInit() {
        while (navx.isCalibrating()) {
            try { Thread.sleep(50); } catch (final InterruptedException e) {}
        }
        navx.zeroYaw(); // zera getAngle() tambem, pois zeroYaw() reseta a referencia de angulo acumulado

        giroIniciado   = false;
        ciclosEstaveis = 0;

        // Exemplo: girar 450 graus (mais de uma volta) a partir do zero
        iniciarGiro(450.0);
    }

    // =========================================================
    @Override
    public void autonomousPeriodic() {
        final boolean ok = executarGiro();
        if (ok) {
            stopMotors();
            SmartDashboard.putString("Nav/Status", "GIRO_CONCLUIDO");
        }
    }

    // =========================================================
    //  Prepara um novo giro relativo de X graus (positivo = sentido horario)
    // =========================================================
    private void iniciarGiro(final double grausRelativos) {
        anguloBase     = navx.getAngle();
        giroAlvo       = grausRelativos;
        giroIniciado   = true;
        ciclosEstaveis = 0;
    }

    // =========================================================
    //  executarGiro - usa navx.getAngle() (continuo) ate atingir o alvo
    //  retorna true quando concluido
    // =========================================================
    private boolean executarGiro() {

        final double anguloAtual = navx.getAngle(); // continuo, sem limite -180/180
        final double girado      = anguloAtual - anguloBase;
        final double erro        = giroAlvo - girado;

        SmartDashboard.putNumber("Giro/anguloAtual", anguloAtual);
        SmartDashboard.putNumber("Giro/girado", girado);
        SmartDashboard.putNumber("Giro/erro_graus", erro);
        SmartDashboard.putNumber("Giro/ciclosEstaveis", ciclosEstaveis);

        if (Math.abs(erro) < TOLERANCE_DEG) {
            ciclosEstaveis++;
            stopMotors();
            if (ciclosEstaveis >= CICLOS_ESTAVEIS) return true;
            return false;
        }

        ciclosEstaveis = 0;
        final double sinal = Math.signum(erro);
        motor0.set(sinal * VEL_GIRO);
        motor2.set(sinal * VEL_GIRO);
        motor3.set(sinal * VEL_GIRO);
        return false;
    }

    // =========================================================
    //  Movimento simples para frente/tras
    // =========================================================
    private void mover(final String dir) {
        switch (dir) {
            case "F":
                motor0.set(0.0); motor2.set(-VEL_MOVE); motor3.set(VEL_MOVE);
                break;
            case "B":
                motor0.set(0.0); motor2.set(VEL_MOVE);  motor3.set(-VEL_MOVE);
                break;
            default:
                stopMotors();
        }
    }

    // =========================================================
    //  Helpers
    // =========================================================
    private void stopMotors() {
        motor0.set(0.0); motor2.set(0.0); motor3.set(0.0);
    }

    // =========================================================
    @Override
    public void disabledInit() {
        stopMotors();
        led1.set(false); led2.set(true);
    }

    @Override
    public void disabledPeriodic() { stopMotors(); }
}