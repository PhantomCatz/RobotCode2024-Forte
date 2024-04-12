
package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.RobotMode;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

public class SubsystemCatzLED extends SubsystemBase {
    private static SubsystemCatzLED instance = new SubsystemCatzLED();
    public static SubsystemCatzLED getInstance() {
        return instance;
    }

    public enum ModeColors {
        Climb(RobotMode.CLIMB, Color.kWhite),
        Amp(RobotMode.AMP, Color.kPurple),
        Speaker(RobotMode.SPEAKER, Color.kYellow),
        Hoard(RobotMode.HOARD, Color.kGreen);

        public RobotMode robotMode;
        public Color color;
        private ModeColors(RobotMode robotMode, Color color){
            this.robotMode = robotMode;
            this.color = color;
        }
    }

    public enum LEDMode {
        Solid,
        Blink,
        Flow;
    }

    private final int LED_PWM_PORT = 2; //2 
    private final int LED_COUNT = 17; //half
    private final double LED_PERIOD = 0.35;

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private Color[] colors = new Color[LED_COUNT];
    private Timer timer;
    public LEDMode ledMode = LEDMode.Solid;
    public boolean isAligned = false;

    private SubsystemCatzLED() {
        led = new AddressableLED(LED_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LED_COUNT*2);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
        timer.start();
        colorSolid(Color.kBlack);
    }

    public void colorSolid(Color color) {
        for (int i = 0; i < LED_COUNT; i++) {
            colors[i] = color;
        }
    }

    public void colorRainbow(){
        for(int i = 0; i < LED_COUNT; i++){
            colors[i] = Color.fromHSV((int)((double)i/LED_COUNT * 180), 255, 255);
        }
    }

    public Color[] getColors() {
        Color[] currentColors = new Color[LED_COUNT];
        if(ledMode == LEDMode.Solid){
            for(int i=0; i<LED_COUNT; i++){
                currentColors[i] = colors[i];
            }
        }
        if(ledMode == LEDMode.Blink){
            if(timer.get()/LED_PERIOD%2 < 1){
                for(int i=0; i<LED_COUNT; i++){
                    currentColors[i] = colors[i];
                }
            } else {
                for(int i=0; i<LED_COUNT; i++){
                    currentColors[i] = Color.kBlack;
                }
            }
        }
        if(ledMode == LEDMode.Flow){
            for(int i=0; i<LED_COUNT; i++){
                currentColors[i] = colors[(int)(i+timer.get()/LED_PERIOD)%LED_COUNT];
            }
        }
        return currentColors;
    }

    @Override
    public void periodic(){
        if(isAligned){
            colorRainbow();
            ledMode = LEDMode.Flow;
        } else {
            for(ModeColors mode: ModeColors.values()){
                if(mode.robotMode == CatzConstants.currentRobotMode){ 
                    colorSolid(mode.color);
                    if((mode.robotMode == RobotMode.SPEAKER && SubsystemCatzShooter.getInstance().shooterLoadBeamBrkBroken()) || (mode.robotMode != RobotMode.SPEAKER && SubsystemCatzIntake.getInstance().getIntakeLoadBeamBreakBroken())){
                        ledMode = LEDMode.Blink;
                    } else {
                        ledMode = LEDMode.Solid;
                    }
                }
            }
        }

        for(int i=0; i<LED_COUNT; i++){
            ledBuffer.setLED(i, colors[i]);
        }
        for(int i=LED_COUNT; i<LED_COUNT*2; i++){
            ledBuffer.setLED(i, colors[LED_COUNT*2-i-1]);
        }
        led.setData(ledBuffer);
    }
}