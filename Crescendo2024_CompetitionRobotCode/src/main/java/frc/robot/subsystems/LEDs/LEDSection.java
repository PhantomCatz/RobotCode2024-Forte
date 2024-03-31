package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.util.Color;

public class LEDSection {
    public enum LEDMode {
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
    private Color[] ledColors2;
    private LEDMode ledMode = LEDMode.Solid;

    public LEDSection(int ledCount) {
        LED_COUNT = ledCount;
        ledColors = new Color[LED_COUNT];
        ledColors2 = new Color[LED_COUNT];
    }

    public void setIteration(int iteration) {
        this.iteration = iteration;
    }

    public void setUpdatePeriod(double second){
        this.iteration = (int)Math.round(second*1000/20);
    }

    public void setMode(LEDMode mode) {
        ledMode = mode;
    }

    public void colorSolid(Color color) {
        for (int i = 0; i < LED_COUNT; i++) {
            ledColors[i] = color;
        }
    }

    public void colorAlternating(Color color1, Color color2) {
        for (int i = 0; i < LED_COUNT; i++) {
            if (i % 2 == 0) {
                ledColors[i] = color1;
            } else {
                ledColors[i] = color2;
            }
        }
    }

    public void colorRainbow() {
        for (int i = 0; i < LED_COUNT; i++) {
            ledColors[i] = Color.fromHSV((int) ((double) i / LED_COUNT * 180), 255, 255);
        }
    }

    public void colorSolid2(Color color) {
        for (int i = 0; i < LED_COUNT; i++) {
            ledColors2[i] = color;
        }
    }

    public void colorAlternating2(Color color1, Color color2) {
        for (int i = 0; i < LED_COUNT; i++) {
            if (i % 2 == 0) {
                ledColors2[i] = color1;
            } else {
                ledColors2[i] = color2;
            }
        }
    }

    public void colorRainbow2() {
        for (int i = 0; i < LED_COUNT; i++) {
            ledColors2[i] = Color.fromHSV((int) ((double) i / LED_COUNT * 180), 255, 255);
        }
    }

    public Color[] getColors() {
        currentIteration++;

        if (currentIteration >= iteration) {
            currentIteration = 1;

            if (alt > 200000) {
                alt = 1;
            }

            alt++;
            
            switch (ledMode) {
                case Solid:
                    return ledColors;

                case Blink:
                    if (alt % 2 == 0) {
                        return ledColors;
                    } else {
                        return new Color[LED_COUNT];
                    }

                case Alternating:
                    if (alt % 2 == 0) {
                        return ledColors;
                    } else {
                        return ledColors2;
                    }

                case Flow:
                    for (int i = 0; i < LED_COUNT; i++) {
                        ledColors[i] = ledColors[(i + 1) % LED_COUNT];
                    }
                    return ledColors;
            }    
        }
        return ledColors;
    }
}