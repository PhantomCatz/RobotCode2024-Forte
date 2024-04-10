package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSection {
    public enum LEDMode {
        Solid,
        Blink,
        BlinkWithAlternate,
        Alternate,
        Flow;
    }


    private final int LED_COUNT;
    private final double BLINK_FREQ = 2;
    private final double FLOW_FREQ  = 0.5;

    private Color[] baseColors;
    private Timer timer;
    public LEDMode ledMode = LEDMode.Solid;

    private boolean alreadyRainbow = false;

    public LEDSection(int ledCount) {
        timer = new Timer();
        LED_COUNT = ledCount;
        baseColors = new Color[LED_COUNT];
        timer.reset();
        timer.start();
    }

    private boolean canBlink(){
        return (int)(timer.get() * BLINK_FREQ)%2 == 0;
    }

    private boolean canFlow(){
        return (int)(timer.get() * FLOW_FREQ)%2 == 0;
    }

    public void colorSolid(Color color) {
        for (int i = 0; i < LED_COUNT; i++) {
            baseColors[i] = color;
        }
        alreadyRainbow = false;
    }

    public void colorRainbow(){
        if(!alreadyRainbow){
            for(int i = 0; i < LED_COUNT; i++){
                baseColors[i] = Color.fromHSV((int)((double)i/LED_COUNT * 180), 255, 255);
            }
            alreadyRainbow = true;
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
        alreadyRainbow = false;
    }

    public Color[] getColors() {
        Color[] currentColors = new Color[LED_COUNT];
        boolean blink = canBlink();
        boolean flow = canFlow();
        for(int i=0; i<LED_COUNT; i++){
            switch(ledMode){
                case Solid:
                    currentColors[i] = baseColors[i];
                break;

                case Alternate:
                    if(blink && i%2 == 0){
                        currentColors[i] = Color.kBlack;
                    } else {
                        currentColors[i] = baseColors[i];
                    }
                break;

                case Blink:
                    if(blink){
                        currentColors[i] = Color.kBlack;
                    } else {
                        currentColors[i] = baseColors[i];
                    }
                break;

                case BlinkWithAlternate:
                    if(blink){
                    currentColors[i] = baseColors[i+1];
                    } else {
                        currentColors[i] = baseColors[i];
                    }
                break;

                case Flow:
                    if(flow){
                        currentColors[i] = baseColors[(i+1)%LED_COUNT];
                    }
                    
                break;
            }
        }
        return currentColors;
    }
}