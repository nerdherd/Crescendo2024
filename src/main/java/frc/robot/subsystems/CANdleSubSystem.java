package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleSubSystem extends SubsystemBase {
    private final CANdle CANdle = new CANdle(Constants.CANdleConstants.CANdleID, "rio");
    private final int ledCount = 100;
    // private CommandPS4Controller joystick;
    
    private Animation animation = null;
    private AnimationTypes currentAnimation = AnimationTypes.Rainbow;
    private AnimationTypes lastAnimation = AnimationTypes.Rainbow;
    
    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        StrobeYellow,
        StrobeGreen,
        StrobeRed,
        Twinkle,
        TwinkleOff,
        SetAll
    }

    public enum Status {
        DISABLED,
        TELEOP,
        AUTO,
        DISCONNECTED,
        HAS_TARGET,// Apriltag detected
        HASNOTE, // Note is in indexer
        SHOTREADY, // Ready to shoot
        LASTSTATUS // To go back to previous animation
    }

    private boolean isSolidColor = false;


    public CANdleSubSystem() {
        // this.joystick = joy;
        // changeAnimation(AnimationTypes.SetAll);
        CANdle.clearAnimation(0);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        CANdle.configAllSettings(configAll, 100);
    }

    private void setColor(int r, int g, int b) {
        CANdle.clearAnimation(0);
        // changeAnimation(AnimationTypes.SetAll);
        CANdle.setLEDs(r, g, b);
        isSolidColor = true;
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return CANdle.getBusVoltage(); }
    public double get5V() { return CANdle.get5VRailVoltage(); }
    public double getCurrent() { return CANdle.getCurrent(); }
    public double getTemperature() { return CANdle.getTemperature(); }
    public void configBrightness(double percent) { CANdle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { CANdle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { CANdle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { CANdle.configStatusLedState(offWhenActive, 0); }

    public void setStatus(Status status){
        switch(status)
        {
            case DISABLED:
                changeAnimation(AnimationTypes.Rainbow);
                break;
            case TELEOP:
                setColor(100, 65, 0);
                break;
            case AUTO:
                changeAnimation(AnimationTypes.Larson);
                break;
            case DISCONNECTED:
                changeAnimation(AnimationTypes.SingleFade);
                break;
            case HAS_TARGET:
                changeAnimation(AnimationTypes.StrobeYellow);
                break;
            case HASNOTE:
                changeAnimation(AnimationTypes.StrobeRed);
                // setColor(0, 255,    0);
                break;
            case SHOTREADY:
                changeAnimation(AnimationTypes.StrobeGreen);
                break;
            case LASTSTATUS:
                changeAnimation(lastAnimation);
                break;
        }
    }

    public void changeAnimation(AnimationTypes newAnimation) {
        lastAnimation = currentAnimation;
        currentAnimation = newAnimation;
        
        CANdle.clearAnimation(0);
        isSolidColor = false;

        switch(newAnimation)
        {
            case ColorFlow:
                animation = new ColorFlowAnimation(128, 20, 70, 0, 0.7, ledCount, Direction.Forward);
                break;
            case Fire:
                animation = new FireAnimation(0.5, 0.7, ledCount, 0.7, 0.5);
                break;
            case Larson:
                animation = new LarsonAnimation(0, 255, 46, 0, 1, ledCount, BounceMode.Front, 3);
                break;
            case Rainbow:
                animation = new RainbowAnimation(1, 0.7, ledCount);
                break;
            case RgbFade:
                animation = new RgbFadeAnimation(0.7, 0.4, ledCount);
                break;
            case SingleFade:
                animation = new SingleFadeAnimation(255, 0, 0, 0, 0.5, ledCount);
                break;
            case StrobeYellow:
                animation = new StrobeAnimation(100, 65, 0, 0, 0.5, ledCount);
                break;
            case StrobeGreen:
                animation = new StrobeAnimation(0, 255, 0, 0, 0.25, ledCount);
                break;
            case StrobeRed:
                animation = new StrobeAnimation(255, 0, 0, 0, 0.25, ledCount);
                break;
            case Twinkle:
                animation = new TwinkleAnimation(30, 70, 60, 0, 0.4, ledCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                animation = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, ledCount, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                animation = null;
                break;
        }
        SmartDashboard.putString("Animation", currentAnimation.toString());
    }

    @Override
    public void periodic() {
        if (isSolidColor) {
            return;
        }

        if(animation == null) {
            changeAnimation(AnimationTypes.SingleFade);
        } else {
            CANdle.animate(animation);
        }
    }
}