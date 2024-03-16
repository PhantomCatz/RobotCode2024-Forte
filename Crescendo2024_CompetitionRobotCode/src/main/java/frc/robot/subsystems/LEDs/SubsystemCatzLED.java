package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.LEDSection.LEDMode;
// import frc.robot.Robot.gameModeLED;
import frc.robot.subsystems.intake.SubsystemCatzIntake;


public class SubsystemCatzLED extends SubsystemBase {

    private static SubsystemCatzLED instance = new SubsystemCatzLED();

    private final int LED_PWM_PORT = 2; //2 soon on friday comp
    private final int LED_COUNT_HALF = 17; //half 18
    private final int LED_EDGE = 4; //4

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public LEDSection top = new LEDSection(LED_EDGE);
    public LEDSection mid = new LEDSection(LED_COUNT_HALF-2*LED_EDGE);
    public LEDSection bot = new LEDSection(LED_EDGE);

    private SubsystemCatzLED() {
        led = new AddressableLED(LED_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LED_COUNT_HALF*2);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        top.colorSolid(Color.kBlack);
        top.colorSolid2(Color.kBlack);

        mid.colorSolid(Color.kBlack);
        mid.colorSolid2(Color.kBlack);

        bot.colorSolid(Color.kBlack);
        bot.colorSolid2(Color.kBlack);
    }

    public static SubsystemCatzLED getInstance() {
        return instance;
    }

    @Override
    public void periodic(){
        Color[] ledColors = new Color[LED_COUNT_HALF];
        Color[][] sectionColors = {bot.getColors(),mid.getColors(),top.getColors()};

        int index = 0;
        for(Color[] section: sectionColors){
        for(Color color: section){
            ledColors[index] = color;
            index++;
        }
    }

    for(int i=0; i<LED_COUNT_HALF; i++){
        Color color = ledColors[i];
        Color opColor = ledColors[LED_COUNT_HALF-i-1];
        if(color == null){
            color = Color.kBlack;
        }
        if(opColor == null){
            opColor = Color.kBlack;
        }

        ledBuffer.setLED(i, color);
        ledBuffer.setLED(i+LED_COUNT_HALF, opColor);
    }
        led.setData(ledBuffer);
        // for(int i=0; i<LED_COUNT_HALF;i++){
        // System.out.println(ledColors[i]);
        // }
    }

    public void signalHumanPlayerAMP() {
        top.colorSolid(Color.kRed);
        mid.colorSolid(Color.kRed);
        bot.colorSolid(Color.kRed);
        top.setMode(LEDMode.Blink);
        mid.setMode(LEDMode.Blink);
        bot.setMode(LEDMode.Blink);
    }
}