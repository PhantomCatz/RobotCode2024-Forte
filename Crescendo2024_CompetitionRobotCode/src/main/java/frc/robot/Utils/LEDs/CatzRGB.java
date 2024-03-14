package frc.robot.Utils.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.Robot.gameModeLED;

@SuppressWarnings("unused")
public class CatzRGB 
{
    enum LEDSections
    {
        //dummy endpoints 65 total 34 left 31 right
        IntakeL(31, 32),   
        ArmL(22, 23),
        ElevatorL(18, 19),

        IntakeR(35, 36),   
        ArmR(40, 41),
        ElevatorR(44, 45);

        //endpoints are inclusive
        private int start;
        private int end;
        LEDSections(int start, int end)
        {
            this.start = start;
            this.end = end;
        }
    } 

    private final int LED_COUNT = 38; //54 //65
    private final int LED_PWM_PORT = 2;
    private final double FLOW_PERIOD = 2.0;
    public final Color PHANTOM_SAPPHIRE = new Color(15, 25, 200); 

    int iterationCounter = 0;

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private Thread flowThread;

    private int flowFirst;
    private int flowLast;
    private Color flowMovingColor;
    private Color flowBackgroundColor;
    private boolean flowEnabled = false;
    private boolean rainbowEnabled = false;

    private final int FIRST_HALF_LED_COUNT = LED_COUNT / 2 - 1;
    private final int SECOND_HALF_LED_COUNT = LED_COUNT / 2;
    private final int LAST_LED_COUNT = LED_COUNT - 1;

    private final double THREAD_PERIOD = FLOW_PERIOD / LED_COUNT;

    public CatzRGB() {
        led = new AddressableLED(LED_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LED_COUNT);
        led.setLength(LED_COUNT);
        led.setData(ledBuffer); //maybe is this null?
        led.start();

        threadInit();
    }

    private void threadInit(){//TBD issue with thread...apparent syntax issue?
        flowThread = new Thread(() -> {
            while(true) { 
                RainbowPeriodic();
                FlowPeriodic();
                Timer.delay(THREAD_PERIOD);
            }
        });

        flowThread.start();
    }

    public ColorMethod doNothing = (color) -> {}; //by using this special syntax called lambda, you essentially create a new instance of the ColorMethod and override its function "execute".
                                                  //this is a short verson of:
                                                  /*
                                                   * public ColorMethod doNothing = new ColorMethod() {
                                                   *                                    @Override
                                                   *                                    public void execute(Color... param){
                                                   * 
                                                   *                                    }
                                                   *                                }
                                                   * 
                                                   * but since there is only one function in the interface, you can shorten it to: public ColorMethod doNothing = (color) -> {}
                                                   * 
                                                   * the "(color)"" is the parameter of the "execute" method
                                                   */
    
    //why are there two "fillLEDBuffer" methods?

    //endpoints are inclusive
    public void fillLEDBuffer(int first, int last, Color color) {
        rainbowEnabled = false;
        flowEnabled = false;
        for(int i=first; i<=last; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public void fillLEDBuffer(Color color){
        rainbowEnabled = false;
        flowEnabled = false;
        for(int i=0; i<LED_COUNT; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public ColorMethod oneColorFill = (color) -> {
        fillLEDBuffer(color[0]);
    };

    public ColorMethod oneColorFillAllianceColor = (color) -> {
        fillLEDBuffer(driverStationAllianceToAllianceColor(DriverStation.getAlliance().get()));
    };

    public ColorMethod startFlowingRainbow = (color) -> {
        if(rainbowEnabled){
            return;
        }

        flowEnabled = false;
        rainbowEnabled = false;
        for(int i=0; i<LED_COUNT; i++) {
            ledBuffer.setHSV(i, 180 * i/LED_COUNT, 255, 255);
        }
        rainbowEnabled = true;
    };

    private void RainbowPeriodic() {
        if(rainbowEnabled) {
            Color tempColor = ledBuffer.getLED(LED_COUNT-1);

            for(int i=LED_COUNT-1; i>0; i--) {
                ledBuffer.setLED(i, ledBuffer.getLED(i-1));
            }

            ledBuffer.setLED(0, tempColor);
        }
    }

    public ColorMethod startFlowing = (color) -> {
        if(flowEnabled && flowMovingColor == color[0] && flowBackgroundColor == color[1]){
            return;
        }

        flowEnabled = false;
        rainbowEnabled = false;
        flowFirst = -1; //do not question //???

        flowMovingColor = color[0];
        flowBackgroundColor = color[1];

        fillLEDBuffer(0, FIRST_HALF_LED_COUNT, color[0]);
        fillLEDBuffer(SECOND_HALF_LED_COUNT, LAST_LED_COUNT, color[1]);
        flowEnabled = true;
    };

    private void FlowPeriodic() {
        //does not work for flowing sections that are only 1 LED long
        if(flowEnabled){
            flowFirst = (flowFirst + 1) % LED_COUNT;
            flowLast = (flowFirst + LED_COUNT/2 - 1) % LED_COUNT;

            ledBuffer.setLED(flowFirst, ledBuffer.getLED((flowFirst + LED_COUNT - 1) % LED_COUNT));
            ledBuffer.setLED((flowLast + 1) % LED_COUNT, ledBuffer.getLED(flowLast));
        }
    }

    public void LEDPeriodic()
    {
        if(Robot.currentGameModeLED == gameModeLED.TeleOp){
            fillLEDBuffer(Robot.currentGamePiece.color);
            fillLEDBuffer(LEDSections.ArmL.start, LEDSections.ArmL.end, Robot.armControlMode.color);
            fillLEDBuffer(LEDSections.IntakeL.start, LEDSections.IntakeL.end, Robot.intakeControlMode.color);
            fillLEDBuffer(LEDSections.ElevatorL.start, LEDSections.ElevatorL.end, Robot.elevatorControlMode.color);
            fillLEDBuffer(LEDSections.ArmR.start, LEDSections.ArmR.end, Robot.armControlMode.color);
            fillLEDBuffer(LEDSections.IntakeR.start, LEDSections.IntakeR.end, Robot.intakeControlMode.color);
            fillLEDBuffer(LEDSections.ElevatorR.start, LEDSections.ElevatorR.end, Robot.elevatorControlMode.color);
        }
        else {
            Robot.currentGameModeLED.method.execute(Robot.currentGameModeLED.color);
        }

        led.setData(ledBuffer);
    }

    public Color driverStationAllianceToAllianceColor(DriverStation.Alliance color){
        if(color == DriverStation.Alliance.Blue){
            return Color.kBlue;
        }
        else{
            return Color.kRed;
        }
    }


    private double timer(double seconds){ // in seconds; converts time to iteration counter units
        //System.out.println(Math.round(seconds/LOOP_CYCLE_MS) + 1);
        return Math.round(seconds/0.02) + 1;
    }
    
    private void ampModeWithNote() {
        fillLEDBuffer(Color.kYellow);
        led.setData(ledBuffer);
    }

    private void speakerModeWithNote() {
        fillLEDBuffer(Color.kOrange);
        led.setData(ledBuffer);
    }

    private void climbModeWithNote() {
        fillLEDBuffer(Color.kBlue);
        led.setData(ledBuffer);
    }

    private void hoardModeWithNote() {
        fillLEDBuffer(Color.kWhite);
        led.setData(ledBuffer);
    }

    private void ampModeWithoutNote() {
        iterationCounter++;
        if(iterationCounter <= timer(0.5)) {
            if(ledBuffer.getLED(0) == Color.kYellow) {
                fillLEDBuffer(Color.kWhite);
            } else {
                fillLEDBuffer(Color.kYellow);
            }
            led.setData(ledBuffer);
            iterationCounter = 0;
        }
    }

    private void autoAlignReadyToShoot() {
        fillLEDBuffer(Color.kGreen);
        led.setData(ledBuffer);
    }

    private void autoAlignGood_NotRampedUp() {
        iterationCounter++;
        if(iterationCounter <= timer(0.5)) {
            if(ledBuffer.getLED(0) == Color.kGreen) {
                fillLEDBuffer(Color.kBlack);
            } else {
                fillLEDBuffer(Color.kGreen);
            }
            led.setData(ledBuffer);
            iterationCounter = 0;
        }
    }

    private void autoAlignMisalign_Horizontal() {
        fillLEDBuffer(Color.kPurple);
        led.setData(ledBuffer);
    }

    private void autoAlignMisalign_HorizontalAndNotRampedUp() {
        iterationCounter++;
        if(iterationCounter <= timer(0.5)) {
            if(ledBuffer.getLED(0) == Color.kPurple) {
                fillLEDBuffer(Color.kBlack);
            } else {
                fillLEDBuffer(Color.kPurple);
            }
            led.setData(ledBuffer);
            iterationCounter = 0;
        }
    }

    private void autoAlignMisalign_Vertical() {
        fillLEDBuffer(Color.kYellow);
        led.setData(ledBuffer);
    }
    
    private void autoAlignMisalign_VerticalAndNotRampedUp() {
        iterationCounter++;
        if(iterationCounter <= timer(0.5)) {
            if(ledBuffer.getLED(0) == Color.kYellow) {
                fillLEDBuffer(Color.kBlack);
            } else {
                fillLEDBuffer(Color.kYellow);
            }
            led.setData(ledBuffer);
            iterationCounter = 0;
        }
    }

    private void autoAlignTargetNotFound() {
        fillLEDBuffer(Color.kBlue);
        led.setData(ledBuffer);
    }

    private void signalHumanPlayerAmp() {
        iterationCounter++;
        if(iterationCounter <= timer(0.05)) {
            if(ledBuffer.getLED(0) == Color.kRed) {
                fillLEDBuffer(Color.kBlack);
            } else {
                fillLEDBuffer(Color.kRed);
            }
            led.setData(ledBuffer);
            iterationCounter = 0;
        }
    }

    private void inAutonomous() {
        iterationCounter++;
        if(iterationCounter <= timer(0.05)) {
            if(ledBuffer.getLED(0) == Color.kYellow) {
                fillLEDBuffer(Color.kBlack);
            } else {
                fillLEDBuffer(Color.kYellow);
            }
            led.setData(ledBuffer);
            iterationCounter = 0;
        }
    }
}
