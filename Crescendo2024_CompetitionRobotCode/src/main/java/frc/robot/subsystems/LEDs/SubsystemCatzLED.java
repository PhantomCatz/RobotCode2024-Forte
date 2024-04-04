
package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.RobotMode;
import frc.robot.subsystems.LEDs.LEDSection.LEDMode;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

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
        private ModeColors(RobotMode robotMode, Color color){
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
            ledBuffer.setLED(i, ledBuffer.getLED(LED_COUNT_HALF*2-i-1));
        }
        led.setData(ledBuffer);

        if(DriverStation.isTeleop()){
            for(ModeColors mode: ModeColors.values()){
                if(mode.robotMode == CatzConstants.currentRobotMode){
                    if(SubsystemCatzIntake.getInstance().getIntakeLoadBeamBreakBroken()){
                        if(!signalHumanPlayer){
                            top.colorSolid(mode.color); top.ledMode = LEDMode.Solid;
                        }
                            bot.colorSolid(mode.color); bot.ledMode = LEDMode.Solid;
                        if(SubsystemCatzShooter.getInstance().getShooterServoInPos() && SubsystemCatzTurret.getInstance().getTurretInPos() && SubsystemCatzShooter.getInstance().shooterLoadBeamBrkBroken()){
                            mid.colorSolid(Color.kOrange); mid.ledMode = LEDMode.Solid;
                        } else {
                            mid.colorSolid(Color.kOrange); mid.ledMode = LEDMode.Blink;
                        }
                    } else {
                        if(!signalHumanPlayer){
                            top.colorAlternating(mode.color, Color.kWhite); top.ledMode = LEDMode.Alternate;
                        }
                        bot.colorAlternating(mode.color, Color.kWhite); bot.ledMode = LEDMode.Alternate;
                        mid.colorAlternating(mode.color, Color.kWhite); mid.ledMode = LEDMode.Alternate;
                    }
                }
            }
        }
    }

    public void signalHumanPlayerAMP(boolean signal) {
        signalHumanPlayer = signal;
        top.colorSolid(Color.kGreen); top.ledMode = LEDMode.Blink;
    }
}
