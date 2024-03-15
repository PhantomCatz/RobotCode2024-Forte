package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.util.Color;

public class LEDSection {
    public enum LEDMode{
        Solid,
        Blink,
        Alternating,
        Flow;
    }

    private final int LED_COUNT;
    private int iteration;
    private int currentIteration = 0;
    private int alt = 0;

    private Color[] ledColors;
    private LEDMode ledMode = LEDMode.Solid;

    private Color color;
    private Color altColor;

    public LEDSection(int ledCount){
        LED_COUNT = ledCount;
        ledColors = new Color[LED_COUNT];
    }

    public void setIteration(int iteration){
        this.iteration = iteration;
    }

    public void setModeNColor(Color color1, Color color2, LEDMode mode){
        color = color1;
        altColor = color2;
        ledMode = mode;
    }

    public void setMode(LEDMode mode){
        ledMode = mode;
    }

    public void colorSolid(Color color){
        for(int i = 0; i < LED_COUNT; i++){
            ledColors[i] = color;
        }
    }

    public void colorAlternating(Color color1, Color color2){
        for(int i = 0; i < LED_COUNT; i++){
            if(i%2==0){
                ledColors[i] = color1;
            }else{
                ledColors[i] = color2;
            }
        }
    }

    public void colorRainbow(){
        ledMode = LEDMode.Flow;
        for(int i = 0; i < LED_COUNT; i++){
            ledColors[i] = Color.fromHSV((int)((double)i/LED_COUNT*180), 255, 255);
        }
    }

    public Color[] getColors(){
        if(currentIteration >= iteration){
            switch(ledMode){
            case Solid:
                colorSolid(color);
            break;

            case Blink:
            if(alt % 2 == 0){
                colorSolid(color);
            }else{
                colorSolid(Color.kBlack);
            }
            break;

            case Alternating:
            if(alt % 2 == 0){
                colorSolid(color);
            }else{
                colorSolid(altColor);
            }
            break;

            case Flow:
            for(int i = 0; i < LED_COUNT; i++){
                ledColors[i] = ledColors[(i+1)%LED_COUNT];
            }
                break;
            }

            currentIteration = 1;

            if(alt > 200000){
                alt = 1;
            }

            alt++;
        }
        currentIteration++;
        return ledColors;
    }
}