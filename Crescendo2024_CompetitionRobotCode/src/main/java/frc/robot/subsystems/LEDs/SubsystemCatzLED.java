package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.RobotMode;
import frc.robot.CatzConstants.NoteDestination;
import frc.robot.CatzConstants.NoteSource;
import frc.robot.subsystems.LEDs.LEDSection.LEDMode;
import frc.robot.subsystems.climb.SubsystemCatzClimb;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;

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
    public boolean signalHumanPlayer = false;

    public enum ModeColors {
        Climb(RobotMode.CLIMB, Color.kBlue),
        Amp(RobotMode.AMP, Color.kPink),
        Speaker(RobotMode.SPEAKER, Color.kYellow),
        Hoard(RobotMode.HOARD, Color.kPurple);

        public RobotMode robotMode;
        public Color color;
        public ModeColors(RobotMode robotMode, Color color){
            this.robotMode = robotMode;
            this.color = color;
        }
    }

    private SubsystemCatzLED() {
        led = new AddressableLED(LED_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LED_COUNT_HALF*2);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        top.colorSolid(Color.kBlack);
        mid.colorSolid(Color.kBlack);
        bot.colorSolid(Color.kBlack);
    }

    public static SubsystemCatzLED getInstance() {
        return instance;
    }

    @Override
    public void periodic(){
        Color[][] sectionColors = {bot.getColors(), mid.getColors(), top.getColors()};

        int index = 0;
        for(Color[] section: sectionColors){
            for(Color color: section){
                ledBuffer.setLED(index, color);
                index++;
            }
        }

        for(int i=LED_COUNT_HALF; i<LED_COUNT_HALF*2; i++){
            ledBuffer.setLED(i, LED_COUNT_HALF*2-i-1);
        }
        led.setData(ledBuffer);

        if(DriverStation.isTeleop()){
            for(ModeColors mode: ModeColors.values()){
                if(mode.robotMode == CatzConstants.currentRobotMode){
                    if(SubsystemCatzIntake.getInstance().getIntakeBeamBreakBroken()){
                        if(!signalHumanPlayer){
                            top.colorSolid(mode.color); top.mode = LEDMode.Solid;
                        }
                        bot.colorSolid(mode.color); bot.mode = LEDMode.Solid;
                        if(SubsystemCatzTurret.getInstance().getTurretInPos()){
                            mid.colorSolid(Color.kOrange); mid.mode = LEDMode.Solid;
                        } else {
                            mid.colorSolid(Color.kOrange); mid.mode = LEDMode.Blink;
                        }
                    } else {
                        if(!signalHumanPlayer){
                            top.colorAlternating(mode.color, Color.kWhite); top.mode = LEDMode.Alternate;
                        }
                        bot.colorAlternating(mode.color, Color.kWhite); bot.mode = LEDMode.Alternate;
                        mid.colorAlternating(mode.color, Color.kWhite); mid.mode = LEDMode.Alternate;
                    }
                }
            }
        }
    }

    public void signalHumanPlayerAMP(boolean signal) {
        signalHumanPlayer = signal;
        top.colorSolid(Color.kGren); top.mode = LEDMode.Blink;
    }
}