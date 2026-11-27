package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.*;

/** 控制所有 CANdle 的燈效。ID 與燈條長度設定在 {@link frc.robot.Constants.LEDConstants}。 */
public class LEDSubsystem extends SubsystemBase {
    /* ===== 常用顏色 ===== */
    public static final RGBWColor OFF    = new RGBWColor(0, 0, 0);
    public static final RGBWColor RED    = new RGBWColor(255, 0, 0);
    public static final RGBWColor GREEN  = new RGBWColor(0, 255, 0);
    public static final RGBWColor BLUE   = new RGBWColor(0, 0, 255);
    public static final RGBWColor YELLOW = new RGBWColor(255, 160, 0);
    public static final RGBWColor PURPLE = new RGBWColor(160, 0, 255);
    public static final RGBWColor WHITE  = new RGBWColor(255, 255, 255);

    private final CANdle[] candles;
    private final EmptyAnimation clearSlot0 = new EmptyAnimation(0);

    public LEDSubsystem(CANBus bus) {
        candles = new CANdle[kCandleIds.length];

        var config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;  
        config.LED.BrightnessScalar = kBrightness;

        for (int i = 0; i < kCandleIds.length; i++) {
            candles[i] = new CANdle(kCandleIds[i], bus);
            candles[i].getConfigurator().apply(config);
        }
        setDefaultCommand(off());
    }

    /** 同一個 request 送到全部 CANdle。 */
    private void applyAll(ControlRequest request) {
        for (CANdle candle : candles) {
            candle.setControl(request);
        }
    }

    /** 清掉動畫 slot 0，再顯示純色（純色不會覆蓋正在跑的動畫，所以要先清）。 */
    private void showSolid(RGBWColor color) {
        applyAll(clearSlot0);
        applyAll(new SolidColor(kStartIndex, kEndIndex).withColor(color));
    }

    /** 開始時套用 request，結束時關燈。動畫只需送一次，CANdle 會自己跑。 */
    private Command runRequest(ControlRequest request, String name) {
        return startEnd(
                () -> {
                    applyAll(clearSlot0);
                    applyAll(request);
                },
                () -> showSolid(OFF))
            .ignoringDisable(true)
            .withName("LED " + name);
    }

    /**
     * 給自動模式 (PathPlanner Named Command) 用：啟動燈效後「立即結束」，不會卡住路徑。
     * 燈效會在背景一直跑，直到被下一個 LED 指令取代。
     */
    public Command trigger(Command effect) {
        return Commands.runOnce(() -> CommandScheduler.getInstance().schedule(effect))
            .ignoringDisable(true)
            .withName("Trigger " + effect.getName());
    }

    /* ===================== 對外的燈效 Command ===================== */

    /** 關燈（預設指令）。 */
    public Command off() {
        return run(() -> {}).beforeStarting(() -> showSolid(OFF))
            .ignoringDisable(true).withName("LED Off");
    }

    /** 純色恆亮。 */
    public Command solid(RGBWColor color) {
        return startEnd(() -> showSolid(color), () -> showSolid(OFF))
            .ignoringDisable(true).withName("LED Solid");
    }

    /** 呼吸燈。frameRateHz 越高呼吸越快 (約 2~1000)。 */
    public Command breathe(RGBWColor color, double frameRateHz) {
        return runRequest(
            new SingleFadeAnimation(kStartIndex, kEndIndex)
                .withSlot(0).withColor(color).withFrameRate(frameRateHz),
            "Breathe");
    }

    /** 爆閃。frameRateHz 越高閃越快。 */
    public Command strobe(RGBWColor color, double frameRateHz) {
        return runRequest(
            new StrobeAnimation(kStartIndex, kEndIndex)
                .withSlot(0).withColor(color).withFrameRate(frameRateHz),
            "Strobe");
    }

    /** 彩虹流動。 */
    public Command rainbow(double frameRateHz) {
        return runRequest(
            new RainbowAnimation(kStartIndex, kEndIndex)
                .withSlot(0).withBrightness(1.0).withFrameRate(frameRateHz),
            "Rainbow");
    }

    /** 跑馬燈（來回掃描）。 */
    public Command larson(RGBWColor color, double frameRateHz) {
        return runRequest(
            new LarsonAnimation(kStartIndex, kEndIndex)
                .withSlot(0).withColor(color).withSize(5).withFrameRate(frameRateHz),
            "Larson");
    }

    /** 顏色流水：單色從一端逐顆點亮到另一端。 */
    public Command colorFlow(RGBWColor color, AnimationDirectionValue direction, double frameRateHz) {
        return runRequest(
            new ColorFlowAnimation(kStartIndex, kEndIndex)
                .withSlot(0).withColor(color).withDirection(direction).withFrameRate(frameRateHz),
            "ColorFlow");
    }

    /**
     * 火焰效果。
     * @param sparking 火花多寡 0~1，越大火越旺
     * @param cooling  降溫快慢 0~1，越大火焰越短
     */
    public Command fire(double sparking, double cooling, double frameRateHz) {
        return runRequest(
            new FireAnimation(kStartIndex, kEndIndex)
                .withSlot(0).withBrightness(1.0)
                .withSparking(sparking).withCooling(cooling)
                .withFrameRate(frameRateHz),
            "Fire");
    }

    /**
     * 閃爍星光：隨機 LED 一閃一閃。
     * @param maxOnProportion 最多幾成 LED 同時亮 0~1
     */
    public Command twinkle(RGBWColor color, double maxOnProportion, double frameRateHz) {
        return runRequest(
            new TwinkleAnimation(kStartIndex, kEndIndex)
                .withSlot(0).withColor(color)
                .withMaxLEDsOnProportion(maxOnProportion).withFrameRate(frameRateHz),
            "Twinkle");
    }

    /**
     * 反向星光：大部分亮著，隨機熄滅。
     * @param maxOnProportion 最多幾成 LED 同時亮 0~1
     */
    public Command twinkleOff(RGBWColor color, double maxOnProportion, double frameRateHz) {
        return runRequest(
            new TwinkleOffAnimation(kStartIndex, kEndIndex)
                .withSlot(0).withColor(color)
                .withMaxLEDsOnProportion(maxOnProportion).withFrameRate(frameRateHz),
            "TwinkleOff");
    }

    /** 整條在紅、綠、藍之間漸變。 */
    public Command rgbFade(double frameRateHz) {
        return runRequest(
            new RgbFadeAnimation(kStartIndex, kEndIndex)
                .withSlot(0).withBrightness(1.0).withFrameRate(frameRateHz),
            "RgbFade");
    }
}
