// package frc.robot;

// import com.studica.frc.Titan;
// import com.studica.frc.MockDS;

// import edu.wpi.cscore.UsbCamera;
// import edu.wpi.first.cameraserver.CameraServer;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.NetworkTableEntry;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DigitalOutput;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class FruitDetection extends TimedRobot {

//     // =========================================================
//     // HARDWARE
//     // =========================================================
//     private Titan         titan;
//     private MockDS        ds;
//     private DigitalInput  btnStart, btnStop;
//     private DigitalOutput led1, led2;

//     private boolean lastStart = false, lastStop = false;

//     // =========================================================
//     // NETWORKTABLES — Dados YOLO do Raspberry Pi
//     // API antiga (2020): NetworkTableEntry
//     // =========================================================
//     private NetworkTableEntry ntDetected;
//     private NetworkTableEntry ntLabel;
//     private NetworkTableEntry ntConfidence;
//     private NetworkTableEntry ntCenterX;
//     private NetworkTableEntry ntCenterY;
//     private NetworkTableEntry ntArea;
//     private NetworkTableEntry ntCount;

//     // =========================================================
//     @Override
//     public void robotInit() {
//         titan    = new Titan(Constants.TITAN_ID);
//         ds       = new MockDS();
//         btnStart = new DigitalInput(Constants.BTN_START);
//         btnStop  = new DigitalInput(Constants.BTN_STOP);
//         led1     = new DigitalOutput(Constants.LEDRun);
//         led2     = new DigitalOutput(Constants.LEDStop);

//         // ---- Câmera USB — stream direto pro Shuffleboard ----
//         new Thread(() -> {
//             UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
//             camera.setResolution(640, 480);
//         }).start();

//         // ---- NetworkTables — recebe dados YOLO do Pi ----
//         NetworkTable table = NetworkTableInstance.getDefault().getTable("FruitDetection");

//         ntDetected   = table.getEntry("detected");
//         ntLabel      = table.getEntry("label");
//         ntConfidence = table.getEntry("confidence");
//         ntCenterX    = table.getEntry("centerX");
//         ntCenterY    = table.getEntry("centerY");
//         ntArea       = table.getEntry("area");
//         ntCount      = table.getEntry("count");
//     }

//     // =========================================================
//     @Override
//     public void robotPeriodic() {
//         // Botões
//         boolean curStart = btnStart.get();
//         boolean curStop  = btnStop.get();
//         if (lastStart && !curStart) { ds.enable();  led1.set(true);  }
//         if (lastStop  && !curStop)  { ds.disable(); led1.set(false); }
//         lastStart = curStart;
//         lastStop  = curStop;

//         // Shuffleboard — dados YOLO vindos do Pi
//         SmartDashboard.putBoolean("Visao/detectada",  ntDetected  .getBoolean(false));
//         SmartDashboard.putString ("Visao/fruta",      ntLabel     .getString(""));
//         SmartDashboard.putNumber ("Visao/confianca",  ntConfidence.getDouble(0.0));
//         SmartDashboard.putNumber ("Visao/centerX",    ntCenterX   .getDouble(0.5));
//         SmartDashboard.putNumber ("Visao/centerY",    ntCenterY   .getDouble(0.5));
//         SmartDashboard.putNumber ("Visao/area",       ntArea      .getDouble(0.0));
//         SmartDashboard.putNumber ("Visao/quantidade", ntCount     .getDouble(0.0));
//     }

//     // =========================================================
//     @Override public void autonomousInit()     {}
//     @Override public void autonomousPeriodic() {}
//     @Override public void disabledInit()       { led1.set(false); led2.set(false); }
//     @Override public void disabledPeriodic()   {}
// }