package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSection {
    public enum LEDMode {
        Solid,
        Blink,
        Alternate,
    }

    private final int LED_COUNT;
    private final double BLINK_FREQ = 2;

    private Color[] baseColors;
    private Timer timer;
    public LEDMode ledMode = LEDMode.Solid;

    public LEDSection(int ledCount) {
        LED_COUNT = ledCount;
        baseColors = new Color[LED_COUNT];
        timer.reset();
        timer.start();
    }

    private boolean canBlink(){
        return (int)(timer.get() * BLINK_FREQ)%2 == 0;
    }

    public void colorSolid(Color color) {
        for (int i = 0; i < LED_COUNT; i++) {
            baseColors[i] = color;
        }
    }

    public void colorAlternating(Color color, Color back){
        for(int i=0; i<LED_COUNT; i++){
            if(i%2 == 0){
                baseColors[i] = back;
            } else {
                baseColors[i] = color;
            }
        }
    }

    public Color[] getColors() {
        Color[] currentColors = new Color[LED_COUNT];
        for(int i=0; i<LED_COUNT; i++){
            if(ledMode == LEDMode.Solid){
                currentColors[i] = baseColors[i];
            }
            if(ledMode == LEDMode.Blink){
                if(canBlink()){
                    currentColors[i] = Color.kBlack;
                } else {
                    currentColors[i] = baseColors[i];
                }
            }
            if(ledMode == LEDMode.Alternate){
                if(canBlink() && i%2 == 0){
                    currentColors[i] = Color.kBlack;
                } else {
                    currentColors[i] = baseColors[i];
                }
            }
        }
        return currentColors;
    }
}