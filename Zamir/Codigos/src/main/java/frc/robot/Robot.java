package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.studica.frc.Titan;
import com.studica.frc.MockDS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.studica.frc.Lidar;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class Robot extends TimedRobot {

    // =========================================================
    // CONFIGURAÇÕES DE INVERSÃO DE MOTORES
    // =========================================================
    private static final boolean INVERT_MOTOR_0 = false;  // Frente Esquerda
    private static final boolean INVERT_MOTOR_1 = false;  // Frente Direita
    private static final boolean INVERT_MOTOR_2 = false;  // Trás Esquerda
    private static final boolean INVERT_MOTOR_3 = false;  // Trás Direita

    // =========================================================
    // CONFIGURAÇÕES DE MOVIMENTO
    // =========================================================
    private static final double VEL_MOVE            = 0.5;      // Velocidade de movimento linear
    private static final double VEL_ROTACAO         = 0.4;      // Velocidade de rotação
    private static final double RAIO_ROBO_MM        = 150.0;    // Distância do eixo ao centro do robô (mm)

    // =========================================================
    // CONFIGURAÇÕES DE SENSORES (Labirinto 300mm)
    // =========================================================
    private static final double DIST_DETECTAR_PAREDE_MM = 200.0;

    // =========================================================
    // CONFIGURAÇÕES DE GIRO PID COM GYRO
    // =========================================================
    private static final double KP_ROTACAO         = 0.1;
    private static final double KI_ROTACAO         = 0.01;
    private static final double KD_ROTACAO         = 0.05;
    private static final double DEADBAND_ROT       = 0.15;
    private static final double TOLERANCE_DEG      = 2.0;
    private static final int    CICLOS_ESTAVEIS    = 8;

    // =========================================================
    // CONFIGURAÇÕES DE CENTRALIZAÇÃO COM GYRO
    // =========================================================
    private static final double KP_CENTRO_GYRO     = 0.08;
    private static final double MARGEM_ERRO_GYRO   = 10.0;
    private static final double MAX_CORRECAO_GYRO  = 0.3;

    // =========================================================
    // CONFIGURAÇÕES DE EXPLORAÇÃO COM LIDAR
    // =========================================================
    private static final double DIST_PAREDE_DETECTADA_MM = 500.0;  // Distância para considerar parede
    private static final double ANGULO_VARREDURA_FRONTAL = 60.0;    // Ângulo de verificação frontal (±30°)
    private static final double BLINK_INTERVALO    = 0.15;

    // =========================================================
    // HARDWARE
    // =========================================================
    private AHRS gyro;
    private Titan titan;
    private Titan.Motor motor0, motor1, motor2, motor3;  // 0=FE, 1=FD, 2=TE, 3=TD
    private Titan.Encoder enc0, enc1, enc2, enc3;

    private MockDS ds;
    private DigitalInput  btnStart, btnStop;
    private DigitalOutput led1, led2;

    // LIDAR
    private Lidar lidar;
    private Lidar.ScanData scanData;
    public boolean scanning = true;

    private boolean lastStart = false, lastStop = false;

    // =========================================================
    // ESTADOS
    // =========================================================
    private enum EstadoRobo {
        PARADO,          // Aguardando início
        EXPLORANDO,      // Executando exploração
        MOVENDO,         // Executando movimento por distância
        GIRANDO,         // Executando rotação
        EXPLORADO,       // Exploração concluída
        PISCANDO         // Piscando LEDs
    }

    private EstadoRobo estado = EstadoRobo.PARADO;

    // =========================================================
    // ODOMETRIA (4 Motores - Diferencial de 4 rodas)
    // =========================================================
    private class Pose {
        public double x, y;
        public double theta;

        public Pose() { this.x = 0.0; this.y = 0.0; this.theta = 0.0; }
        public Pose(double x, double y, double theta) {
            this.x = x; this.y = y; this.theta = theta;
        }

        @Override
        public String toString() {
            return String.format("(%.2f, %.2f, %.1f°)", x, y, theta);
        }
    }

    private Pose poseRobo = new Pose();
    private double[] ultimasDistancias = {0.0, 0.0, 0.0, 0.0};  // Últimas distâncias de cada motor
    private double ultimaRotacao = 0.0;

    private Set<String> celulasMapeadas = new HashSet<>();
    private double tempoUltimaDeteccao = 0.0;
    private boolean emPausaDeteccao = false;

    // =========================================================
    // CONTROLE DE MOVIMENTO
    // =========================================================
    private boolean moveIniciado = false;
    private double distanciaAlvo = 0.0;
    private double velocidadeAtual = 0.0;
    private double distanciaPercorrida = 0.0;
    private double giroAposMovimento = 0.0;  // Giro a executar após movimento

    private boolean giroIniciado = false;
    private int ciclosEstavelGiro = 0;
    private double giroPendente = 0.0;
    private double giroErroIntegral = 0.0;
    private double giroErroAnterior = 0.0;

    // =========================================================
    @Override
    public void robotInit() {
        gyro = new AHRS(SPI.Port.kMXP);

        titan  = new Titan(Constants.TITAN_ID);
        motor0 = titan.getMotor(Constants.MOTOR_0);  // Frente Esquerda
        motor1 = titan.getMotor(Constants.MOTOR_1);  // Frente Direita
        motor2 = titan.getMotor(Constants.MOTOR_2);  // Trás Esquerda
        motor3 = titan.getMotor(Constants.MOTOR_3);  // Trás Direita

        enc0 = titan.getEncoder(Constants.ENCODER_0, Constants.DIST_PER_TICK);
        enc1 = titan.getEncoder(Constants.ENCODER_1, Constants.DIST_PER_TICK);
        enc2 = titan.getEncoder(Constants.ENCODER_2, Constants.DIST_PER_TICK);
        enc3 = titan.getEncoder(Constants.ENCODER_3, Constants.DIST_PER_TICK);

        ds       = new MockDS();
        btnStart = new DigitalInput(Constants.BTN_START);
        btnStop  = new DigitalInput(Constants.BTN_STOP);
        led1     = new DigitalOutput(Constants.LEDRun);
        led2     = new DigitalOutput(Constants.LEDStop);

        // LIDAR Setup
        lidar = new Lidar(Lidar.Port.kUSB1);
        lidar.clusterConfig(50.0f, 5);
        lidar.kalmanConfig(1e-5f, 1e-1f, 1.0f);
        lidar.movingAverageConfig(5);
        lidar.medianConfig(5);
        lidar.jitterConfig(50.0f);
        lidar.enableFilter(Lidar.Filter.kCLUSTER, true);
    }

    @Override
    public void robotPeriodic() {
        final boolean curStart = btnStart.get();
        final boolean curStop  = btnStop.get();

        if (lastStart && !curStart) {
            ds.enable();
            led1.set(true);
        }
        if (lastStop && !curStop) {
            ds.disable();
            led1.set(false);
        }
        lastStart = curStart;
        lastStop  = curStop;

        // Dashboard
        SmartDashboard.putNumber("Pose/x_m",           poseRobo.x);
        SmartDashboard.putNumber("Pose/y_m",           poseRobo.y);
        SmartDashboard.putNumber("Pose/theta_deg",     poseRobo.theta);
        SmartDashboard.putString("Robot/Estado",       estado.toString());
        SmartDashboard.putNumber("Map/celulasMapeadas", celulasMapeadas.size());

        // LIDAR Data
        scanData = lidar.getData();
        if (scanData != null && scanData.distance != null && scanData.distance.length > 180) {
            SmartDashboard.putNumber("LIDAR/Angle_180",  scanData.angle[180]);
            SmartDashboard.putNumber("LIDAR/Distance_180", scanData.distance[180]);
        }
    }

    // =========================================================
    @Override
    public void autonomousInit() {
        while (gyro.isCalibrating()) {
            try { Thread.sleep(10); } catch (final InterruptedException e) {}
        }
        gyro.zeroYaw();

        // Reset encoders
        enc0.reset(); enc1.reset(); enc2.reset(); enc3.reset();

        estado = EstadoRobo.EXPLORANDO;
        poseRobo = new Pose(0.0, 0.0, 0.0);
        celulasMapeadas.clear();
        celulasMapeadas.add("0,0");

        moveIniciado = false;
        giroIniciado = false;
        distanciaAlvo = 0.0;
        velocidadeAtual = 0.0;
        distanciaPercorrida = 0.0;
        giroAposMovimento = 0.0;

        tempoUltimaDeteccao = Timer.getFPGATimestamp();
        led1.set(true);
        led2.set(false);

        lidar.start();
        scanning = true;
    }

    // =========================================================
    @Override
    public void autonomousPeriodic() {
        atualizarOdometria();

        if (estado == EstadoRobo.EXPLORANDO) {
            explorarComLidar();
        }

        SmartDashboard.putNumber("Enc/0", enc0.getDistance());
        SmartDashboard.putNumber("Enc/1", enc1.getDistance());
        SmartDashboard.putNumber("Enc/2", enc2.getDistance());
        SmartDashboard.putNumber("Enc/3", enc3.getDistance());
    }

    // =========================================================
    // EXPLORAÇÃO COM LIDAR
    // Mapeamento de ângulos LIDAR:
    // 0° = Esquerda
    // 90° = Frente
    // 180° = Direita
    // 270° = Trás
    // =========================================================
    private void explorarComLidar() {
        if (scanData == null || scanData.distance == null || scanData.distance.length < 360) {
            // Sem dados LIDAR, parar
            stopMotors();
            return;
        }

        // Verifica parede na frente (ângulos de 60° a 120°, centro em 90°)
        boolean paredeFrente = verificarParedeNoAngulo(scanData, 90.0, ANGULO_VARREDURA_FRONTAL);
        
        // Verifica parede à direita (ângulos de 150° a 210°, centro em 180°)
        boolean paredeDireita = verificarParedeNoAngulo(scanData, 180.0, ANGULO_VARREDURA_FRONTAL);
        
        // Verifica parede à esquerda (ângulos de 330° a 30°, centro em 0°/360°)
        boolean paredeEsquerda = verificarParedeNoAngulo(scanData, 0.0, ANGULO_VARREDURA_FRONTAL);

        SmartDashboard.putBoolean("LIDAR/ParedeFrente", paredeFrente);
        SmartDashboard.putBoolean("LIDAR/ParedeDireita", paredeDireita);
        SmartDashboard.putBoolean("LIDAR/ParedeEsquerda", paredeEsquerda);
        SmartDashboard.putNumber("Move/DistPercorrida", distanciaPercorrida);

        String acaoAtual = "NENHUMA";

        // Lógica de exploração com prioridade
        if (paredeFrente) {
            // Tem parede na frente, precisa se mover antes de girar
            // Primeiro move 0.2m para frente, depois gira
            if (moverDistancia(0.2, VEL_MOVE)) {
                // Movimento de 0.2m completo, agora gira
                if (!paredeDireita) {
                    // Sem parede na direita, vira para direita
                    acaoAtual = "MOVE 0.2m + GIRA -90 DIREITA";
                    if (!giroIniciado) {
                        executarGiro(-90.0);
                    }
                } else if (!paredeEsquerda) {
                    // Sem parede na esquerda, vira para esquerda
                    acaoAtual = "MOVE 0.2m + GIRA +90 ESQUERDA";
                    if (!giroIniciado) {
                        executarGiro(90.0);
                    }
                } else {
                    // Paredes na frente, direita e esquerda - vira 180°
                    acaoAtual = "MOVE 0.2m + GIRA 180";
                    if (!giroIniciado) {
                        executarGiro(180.0);
                    }
                }
            } else {
                // Ainda está se movendo os 0.2m
                acaoAtual = "MOVENDO 0.2m (distância: " + String.format("%.3f", distanciaPercorrida) + "m)";
            }
        } else {
            // Sem parede na frente, pode avançar
            acaoAtual = "ANDA PARA FRENTE";
            moverFrente(VEL_MOVE);
        }

        SmartDashboard.putString("Debug/AcaoAtual", acaoAtual);
    }

    /**
     * Verifica se há parede em um intervalo angular
     */
    private boolean verificarParedeNoAngulo(Lidar.ScanData data, double anguloAlvo, double margem) {
        int count = 0;
        int total = 0;

        for (int i = 0; i < data.distance.length; i++) {
            double angulo = data.angle[i];
            
            // Normaliza ângulo para 0-360
            while (angulo < 0) angulo += 360.0;
            while (angulo >= 360) angulo -= 360.0;

            double alvoNorm = anguloAlvo;
            while (alvoNorm < 0) alvoNorm += 360.0;
            while (alvoNorm >= 360) alvoNorm -= 360.0;

            // Calcula diferença angular circular
            double diff = Math.abs(angulo - alvoNorm);
            if (diff > 180.0) diff = 360.0 - diff;

            // Se está dentro da margem
            if (diff <= margem) {
                total++;
                if (data.distance[i] < DIST_PAREDE_DETECTADA_MM) {
                    count++;
                }
            }
        }

        // Retorna verdadeiro se mais de 50% dos pontos detectam parede
        return total > 0 && (count * 100.0 / total) > 50.0;
    }

    // =========================================================
    // MOVIMENTAÇÃO - 4 MOTORES (Diferencial)
    // =========================================================

    /**
     * Move o robô uma distância específica para frente
     * @param distancia Distância em metros (ex: 0.2 para 20cm)
     * @param velocidade Velocidade (0.0 a 1.0)
     * @return true quando completou a distância
     */
    private boolean moverDistancia(double distancia, double velocidade) {
        if (!moveIniciado) {
            enc0.reset();
            enc1.reset();
            enc2.reset();
            enc3.reset();
            moveIniciado = true;
            distanciaPercorrida = 0.0;
            return false;
        }

        // Calcula média de distância percorrida pelos 4 motores
        double dist0 = Math.abs(enc0.getDistance());
        double dist1 = Math.abs(enc1.getDistance());
        double dist2 = Math.abs(enc2.getDistance());
        double dist3 = Math.abs(enc3.getDistance());
        
        distanciaPercorrida = (dist0 + dist1 + dist2 + dist3) / 4.0;

        // Verifica se atingiu a distância alvo
        if (distanciaPercorrida >= Math.abs(distancia)) {
            stopMotors();
            moveIniciado = false;
            distanciaPercorrida = 0.0;
            return true;  // Movimento completo
        }

        // Correção de rotação (keeps robot straight)
        double erroAngular = normalizeAngle180(-gyro.getAngle());
        double correcaoGiro = clamp(erroAngular * KP_CENTRO_GYRO, -MAX_CORRECAO_GYRO, MAX_CORRECAO_GYRO);

        // Movimento para frente com feedback de gyro
        double vel0 = -velocidade + correcaoGiro;
        double vel1 = velocidade + correcaoGiro;
        double vel2 = -velocidade + correcaoGiro;
        double vel3 = velocidade + correcaoGiro;

        setMotor(0, vel0);
        setMotor(1, vel1);
        setMotor(2, vel2);
        setMotor(3, vel3);

        velocidadeAtual = velocidade;
        return false;  // Ainda está movendo
    }

    /**
     * Move o robô para frente com compensação giratória via gyro
     */
    private void moverFrente(double velocidade) {
        if (!moveIniciado) {
            enc0.reset();
            enc1.reset();
            enc2.reset();
            enc3.reset();
            moveIniciado = true;
        }

        // Correção de rotação (keeps robot straight)
        double erroAngular = normalizeAngle180(-gyro.getAngle());
        double correcaoGiro = clamp(erroAngular * KP_CENTRO_GYRO, -MAX_CORRECAO_GYRO, MAX_CORRECAO_GYRO);

        // Configuração dos motores para movimento frontal
        // Frente Esquerda e Trás Esquerda = velocidade negativa (para trás no eixo Y)
        // Frente Direita e Trás Direita = velocidade positiva (para frente no eixo Y)
        // Motor 0 (Frente Esquerda) = -velocidade + correcao
        // Motor 1 (Frente Direita) = velocidade + correcao
        // Motor 2 (Trás Esquerda) = -velocidade + correcao
        // Motor 3 (Trás Direita) = velocidade + correcao

        double vel0 = -velocidade + correcaoGiro;
        double vel1 = velocidade + correcaoGiro;
        double vel2 = -velocidade + correcaoGiro;
        double vel3 = velocidade + correcaoGiro;

        setMotor(0, vel0);
        setMotor(1, vel1);
        setMotor(2, vel2);
        setMotor(3, vel3);

        velocidadeAtual = velocidade;
    }

    /**
     * Move o robô para trás
     */
    private void moverTras(double velocidade) {
        if (!moveIniciado) {
            enc0.reset();
            enc1.reset();
            enc2.reset();
            enc3.reset();
            moveIniciado = true;
        }

        double erroAngular = normalizeAngle180(-gyro.getAngle());
        double correcaoGiro = clamp(erroAngular * KP_CENTRO_GYRO, -MAX_CORRECAO_GYRO, MAX_CORRECAO_GYRO);

        double vel0 = velocidade + correcaoGiro;
        double vel1 = -velocidade + correcaoGiro;
        double vel2 = velocidade + correcaoGiro;
        double vel3 = -velocidade + correcaoGiro;

        setMotor(0, vel0);
        setMotor(1, vel1);
        setMotor(2, vel2);
        setMotor(3, vel3);

        velocidadeAtual = -velocidade;
    }

    /**
     * Move o robô para a esquerda (movimento lateral)
     */
    private void moverEsquerda(double velocidade) {
        if (!moveIniciado) {
            enc0.reset();
            enc1.reset();
            enc2.reset();
            enc3.reset();
            moveIniciado = true;
        }

        // Movimento lateral: motores diagonais em direções opostas
        // FE e TD = sentido negativo
        // FD e TE = sentido positivo
        setMotor(0, -velocidade);
        setMotor(1, velocidade);
        setMotor(2, velocidade);
        setMotor(3, -velocidade);

        velocidadeAtual = -velocidade;
    }

    /**
     * Move o robô para a direita (movimento lateral)
     */
    private void moverDireita(double velocidade) {
        if (!moveIniciado) {
            enc0.reset();
            enc1.reset();
            enc2.reset();
            enc3.reset();
            moveIniciado = true;
        }

        // Movimento lateral: motores diagonais em direções opostas
        // FE e TD = sentido positivo
        // FD e TE = sentido negativo
        setMotor(0, velocidade);
        setMotor(1, -velocidade);
        setMotor(2, -velocidade);
        setMotor(3, velocidade);

        velocidadeAtual = velocidade;
    }

    /**
     * Rotaciona o robô em torno do seu centro (spin)
     * Positivo = anti-horário, Negativo = horário
     */
    private void rotacionarEmLugar(double omega) {
        if (!giroIniciado) {
            enc0.reset();
            enc1.reset();
            enc2.reset();
            enc3.reset();
            giroIniciado = true;
        }

        // Todos os motores giram em direções alternadas para rotação
        // FE e TE = omega (esquerda superior e direita superior)
        // FD e TD = -omega (direita superior e esquerda inferior)
        setMotor(0, omega);
        setMotor(1, -omega);
        setMotor(2, omega);
        setMotor(3, -omega);
    }

    // =========================================================
    // ODOMETRIA (4 Motores)
    // =========================================================
    private void atualizarOdometria() {
        // Lê distâncias dos 4 encoders
        double dist0 = enc0.getDistance();
        double dist1 = enc1.getDistance();
        double dist2 = enc2.getDistance();
        double dist3 = enc3.getDistance();

        // Calcula deltas
        double delta0 = dist0 - ultimasDistancias[0];
        double delta1 = dist1 - ultimasDistancias[1];
        double delta2 = dist2 - ultimasDistancias[2];
        double delta3 = dist3 - ultimasDistancias[3];

        // Atualiza últimas distâncias
        ultimasDistancias[0] = dist0;
        ultimasDistancias[1] = dist1;
        ultimasDistancias[2] = dist2;
        ultimasDistancias[3] = dist3;

        // Para robô diferencial de 4 rodas (2 esquerda, 2 direita):
        // Distância esquerda = média das rodas esquerda (0 e 2)
        // Distância direita = média das rodas direita (1 e 3)
        double distEsq = (Math.abs(delta0) + Math.abs(delta2)) / 2.0;
        double distDir = (Math.abs(delta1) + Math.abs(delta3)) / 2.0;
        double distMedia = (distEsq + distDir) / 2.0;

        // Calcula mudança de orientação a partir do gyro
        double anguloAtual = gyro.getAngle();
        double deltaAngulo = anguloAtual - ultimaRotacao;

        // Atualiza odometria baseado no movimento linear
        if (Math.abs(distMedia) > 0.01) {
            poseRobo.x += distMedia * Math.cos(Math.toRadians(poseRobo.theta));
            poseRobo.y += distMedia * Math.sin(Math.toRadians(poseRobo.theta));
        }

        // Atualiza orientação
        poseRobo.theta = anguloAtual;
        ultimaRotacao = anguloAtual;

        // Detecta nova célula
        String celulaNova = calcularCelula(poseRobo.x, poseRobo.y);
        if (!celulasMapeadas.contains(celulaNova)) {
            celulasMapeadas.add(celulaNova);
            tempoUltimaDeteccao = Timer.getFPGATimestamp();
            SmartDashboard.putString("Map/celula", celulaNova);
        }
    }

    private String calcularCelula(double x, double y) {
        int cellX = (int) Math.round(x * 1000 / DIST_DETECTAR_PAREDE_MM);
        int cellY = (int) Math.round(y * 1000 / DIST_DETECTAR_PAREDE_MM);
        return cellX + "," + cellY;
    }

    // =========================================================
    // ROTAÇÃO COM PID + GYRO
    // =========================================================
    private boolean executarGiro(final double grausAlvo) {
        if (!giroIniciado) {
            gyro.zeroYaw();
            ciclosEstavelGiro = 0;
            giroErroIntegral = 0.0;
            giroErroAnterior = 0.0;
            giroIniciado = true;
            return false;
        }

        final double anguloAtual = gyro.getAngle();
        final double erro = normalizeAngle180(grausAlvo - anguloAtual);

        SmartDashboard.putNumber("Giro/alvo", grausAlvo);
        SmartDashboard.putNumber("Giro/atual", anguloAtual);
        SmartDashboard.putNumber("Giro/erro", erro);
        SmartDashboard.putNumber("Giro/ciclos", ciclosEstavelGiro);

        // Verifica se chegou perto do alvo
        if (Math.abs(erro) < TOLERANCE_DEG) {
            ciclosEstavelGiro++;
            stopMotors();
            if (ciclosEstavelGiro >= CICLOS_ESTAVEIS) {
                giroIniciado = false;
                gyro.zeroYaw();
                poseRobo.theta = 0.0;
                giroErroIntegral = 0.0;
                giroErroAnterior = 0.0;
                moveIniciado = false;
                return true;
            }
            return false;
        }

        ciclosEstavelGiro = 0;

        // PID para o giro
        giroErroIntegral += erro;
        giroErroIntegral = clamp(giroErroIntegral, -180.0, 180.0);
        double deltErro = erro - giroErroAnterior;
        giroErroAnterior = erro;

        double omega = (KP_ROTACAO * erro) + (KI_ROTACAO * giroErroIntegral) + (KD_ROTACAO * deltErro);
        omega = clamp(omega, -VEL_ROTACAO, VEL_ROTACAO);
        omega = applyDeadband(omega, DEADBAND_ROT);

        rotacionarEmLugar(omega);

        return false;
    }

    // =========================================================
    // UTILITÁRIOS - CONTROLE DE MOTORES
    // =========================================================
    
    /**
     * Define velocidade de um motor com inversão configurável
     */
    private void setMotor(int motorIndex, double speed) {
        speed = clamp(speed, -1.0, 1.0);

        boolean inverted = false;
        Titan.Motor motor = null;

        switch (motorIndex) {
            case 0:
                motor = motor0;
                inverted = INVERT_MOTOR_0;
                break;
            case 1:
                motor = motor1;
                inverted = INVERT_MOTOR_1;
                break;
            case 2:
                motor = motor2;
                inverted = INVERT_MOTOR_2;
                break;
            case 3:
                motor = motor3;
                inverted = INVERT_MOTOR_3;
                break;
        }

        if (motor != null) {
            motor.set(inverted ? -speed : speed);
        }
    }

    private double applyDeadband(final double v, final double db) {
        if (Math.abs(v) < db) return 0.0;
        final double sign = Math.signum(v);
        return sign * clamp((Math.abs(v) - db) / (1.0 - db), 0.0, 1.0);
    }

    private double clamp(final double v, final double min, final double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double normalizeAngle180(double deg) {
        while (deg >  180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }

    private void stopMotors() {
        setMotor(0, 0.0);
        setMotor(1, 0.0);
        setMotor(2, 0.0);
        setMotor(3, 0.0);
    }

    // =========================================================
    @Override
    public void disabledInit() {
        stopMotors();
        estado = EstadoRobo.PARADO;
        moveIniciado = false;
        distanciaAlvo = 0.0;
        velocidadeAtual = 0.0;
        distanciaPercorrida = 0.0;
        giroAposMovimento = 0.0;
        giroIniciado = false;
        ciclosEstavelGiro = 0;
        led1.set(false);
        led2.set(false);
        lidar.stop();
        scanning = false;
    }

    @Override
    public void disabledPeriodic() {
        stopMotors();
        lidar.stop();
        scanning = false;
    }
}