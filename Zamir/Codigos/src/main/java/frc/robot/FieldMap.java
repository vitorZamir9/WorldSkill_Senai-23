package frc.robot;

/**
 * ============================================================
 *  MAPA DO CAMPO - Plano Cartesiano (unidade: centímetros)
 * ============================================================
 *
 *  Y
 *  ^
 *  |
 *  |   [CAMPO]
 *  |
 *  +-----------> X
 *  (0,0) = canto inferior esquerdo do campo
 *
 *  COMO CONFIGURAR:
 *
 *  1. CAMPO: defina largura e altura em cm
 *
 *  2. POSIÇÃO INICIAL: ponto (x, y) em cm + ângulo inicial em graus
 *
 *  3. OBSTÁCULOS: cada obstáculo é um retângulo definido por 4 pontos
 *     Formato: { x_min, y_min, x_max, y_max } em cm
 *     (canto inferior esquerdo -> canto superior direito)
 *     Exemplo:  { 50, 30, 80, 60 } -> retângulo de 30x30cm com canto em (50,30)
 *
 *  4. WAYPOINTS (objetivos sequenciais): lista de pontos (x, y) em cm
 *     O robô vai do ponto inicial -> wp[0] -> wp[1] -> ... -> wp[n]
 *     Opcionalmente: ângulo final desejado ao chegar em cada waypoint (-999 = não girar)
 *
 * ============================================================
 */
public final class FieldMap {

    // =========================================================
    //  DIMENSÕES DO CAMPO (cm)
    // =========================================================
    public static final double CAMPO_LARGURA_CM = 873.0;
    public static final double CAMPO_ALTURA_CM  = 341.0;

    // =========================================================
    //  POSIÇÃO INICIAL DO ROBÔ
    // =========================================================
    public static final double INICIO_X_CM   = 437.0;
    public static final double INICIO_Y_CM   = 170.0;
    public static final double INICIO_ANGULO = -90.0;

    // =========================================================
    //  OBSTÁCULOS { x_min, y_min, x_max, y_max } em cm
    // =========================================================
    public static final double[][] OBSTACULOS = {
        { 428.0, 130.0, 546.0, 341.0 },  // obstáculo central
    };

    // =========================================================
    //  MARGEM DE SEGURANÇA (cm)
    // =========================================================
    public static final double MARGEM_OBSTACULO_CM = 20.0; // aumentada de 15 para 20

    // =========================================================
    //  TOLERÂNCIA DE CHEGADA (cm)
    // =========================================================
    public static final double TOLERANCIA_CHEGADA_CM = 3.0;

    // =========================================================
    //  WAYPOINTS { x_cm, y_cm, angulo_final }
    //
    //  ROTA MANUAL SEGURA para ir de (372, 170.5) até (620, 90):
    //
    //  O obstáculo ocupa X: 408~566 (com margem 20), Y: 110~361
    //  A faixa livre abaixo do obstáculo é Y < 110 cm.
    //
    //  Estratégia:
    //    WP0: desce para Y=90  (eixo Y, livre em X=372)
    //    WP1: avança para X=620 (eixo X, livre em Y=90)
    //
    //  Assim o robô nunca passa pela área do obstáculo.
    // =========================================================
    public static final double[][] WAYPOINTS = {
        { 437.0,  100.0,  -999 }, 
        //{ 100.0,  80.0,  -999 }, 
        { 400.0,  210.0,  -999 },
        { 370.0,  170.0,  -270 },
        //{ 100.0,  90.0,  -999 },

    };

    // =========================================================
    //  Verifica se ponto (px, py) está dentro de algum obstáculo
    // =========================================================
    public static boolean dentroDeObstaculo(final double px, final double py) {
        for (final double[] obs : OBSTACULOS) {
            final double x1 = obs[0] - MARGEM_OBSTACULO_CM;
            final double y1 = obs[1] - MARGEM_OBSTACULO_CM;
            final double x2 = obs[2] + MARGEM_OBSTACULO_CM;
            final double y2 = obs[3] + MARGEM_OBSTACULO_CM;
            if (px >= x1 && px <= x2 && py >= y1 && py <= y2) return true;
        }
        return false;
    }
}