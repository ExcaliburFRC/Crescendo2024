package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Color;

import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static frc.lib.Color.Colors.*;
import static frc.robot.Constants.LedsConstants.LEDS_PORT;
import static frc.robot.Constants.LedsConstants.LENGTH;

// shooterPreparing - Green Blinking
// shooterReady - Green Solid
// intaking - Orange Blinking
//shooting - Red Solid

public class LEDs extends SubsystemBase {
    private final AddressableLED LedStrip = new AddressableLED(LEDS_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);

    private static LEDs instance = null;
    private Random rnd = new Random();

    private int tailIndex = 0;
    private double offset = 0;

    private LEDs() {
        LedStrip.setLength(LENGTH);
        LedStrip.start();

        setDefaultCommand(setPattern(LEDPattern.TRAIN, BLUE.color, TEAM_GOLD.color));
    }

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }
        return instance;
    }

    public Command setPattern(LEDPattern pattern, Color mainColor, Color accentColor) {
        Command command = new InstantCommand();
        Color[] colors = new Color[LENGTH];
        int trainLength = (int) (LENGTH / 5.0);
        final AtomicBoolean invert = new AtomicBoolean(false);

        switch (pattern) {
            case OFF:
                Arrays.fill(colors, OFF.color);
                command = new RunCommand(() -> setLedStrip(colors), this).withName("OFF");
                break;

            case SOLID:
                Arrays.fill(colors, mainColor);
                command = new RunCommand(() -> setLedStrip(colors), this).withName("SOLID: " + mainColor.toString());
                break;

            case ALTERNATING_STATIC:
                for (int i = 0; i < LENGTH; i++) {
                    colors[i] = mainColor;
                    colors[i + 1] = accentColor;
                    i++;
                }
                command = new RunCommand(() -> setLedStrip(colors), this)
                        .withName("ALTERNATING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case ALTERNATING_MOVING:
                AtomicReference<Color> mainAlternatingColor = new AtomicReference<>(mainColor);
                AtomicReference<Color> accentAlternatingColor = new AtomicReference<>(accentColor);
                AtomicReference<Color> tempAlternatingColor = new AtomicReference<>(mainColor);

                command = this.runOnce(() -> {
                            for (int i = 0; i < LENGTH - 1; i++) {
                                colors[i] = mainAlternatingColor.get();
                                colors[i + 1] = accentAlternatingColor.get();
                                i++;
                            }
                            setLedStrip(colors);

                            tempAlternatingColor.set(mainAlternatingColor.get());
                            mainAlternatingColor.set(accentAlternatingColor.get());
                            accentAlternatingColor.set(tempAlternatingColor.get());
                        })
                        .andThen(new WaitCommand(0.25)).repeatedly()
                        .withName("ALTERNATING_MOVING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());

                break;

            case RANDOM:
                command = this.runOnce(() -> {
                            Arrays.fill(colors, new Color(rnd.nextInt(255), rnd.nextInt(255), rnd.nextInt(255)));
                            setLedStrip(colors);
                        }).andThen(new WaitCommand(1))
                        .repeatedly().withName("RANDOM");
                break;

            case BLINKING:
                command = Commands.repeatingSequence(
                        new InstantCommand(() -> {
                            Arrays.fill(colors, mainColor);
                            setLedStrip(colors);
                        }, this),
                        new WaitCommand(0.15),
                        new InstantCommand(() -> {
                            Arrays.fill(colors, accentColor);
                            setLedStrip(colors);
                        }, this),
                        new WaitCommand(0.15)
                ).withName("BLINKING, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case TRAIN:
                command = new InstantCommand(() -> {
                    Arrays.fill(colors, mainColor);
                    for (int j = 0; j < trainLength; j++)
                        colors[MathUtil.clamp(j + tailIndex - 1, 0, LENGTH)] = accentColor;
                    this.tailIndex = invert.get() ? this.tailIndex - 1 : this.tailIndex + 1;
                    if (this.tailIndex == LENGTH - trainLength || this.tailIndex == 0) invert.set(!invert.get());
                    setLedStrip(colors);
                }, this)
                        .andThen(new WaitCommand(0.05)).repeatedly()
                        .withName("TRAIN_BACK_AND_FOURTH, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case TRAIN_CIRCLE:
                command = new InstantCommand(() -> {
                    Arrays.fill(colors, mainColor);
                    for (int j = 0; j < trainLength; j++) colors[stayInBounds(j + tailIndex, LENGTH)] = accentColor;
                    tailIndex++;
                    if (tailIndex == LENGTH) tailIndex = 0;
                    setLedStrip(colors);
                }, this)
                        .andThen(new WaitCommand(0.01)).repeatedly()
                        .withName("TRAIN_CIRCLE, main: " + mainColor.toString() + ", accent: " + accentColor.toString());
                break;

            case RAINBOW:
                AtomicInteger firstHue = new AtomicInteger(0);
                command = this.run(() -> {
                    for (var i = 0; i < LENGTH; i++) {
                        final var hue = (firstHue.get() + (i * 180 / LENGTH)) % 180;
                        buffer.setHSV(i, hue, 255, 128);
                    }
                    firstHue.addAndGet(3);
                    firstHue.set(firstHue.get() % 180);
                });

            default:
                break;
        }

        return command.ignoringDisable(true);
    }

    public Command setPattern(LEDPattern pattern, Color color) {
        return setPattern(pattern, color, OFF.color);
    }


    public Command setLEDsCommand(Color[] colors) {
        return this.runOnce(() -> setLedStrip(colors)).ignoringDisable(true);
    }

    public Command twoStatesCommand(LEDPattern firstPattern, Color firstColor, LEDPattern secondPattern, Color secondColor, Trigger switchStates){
        return setPattern(firstPattern, firstColor).until(switchStates).andThen(setPattern(secondPattern, secondColor));
    }

    public Command deadLineLEDcommand(Command ledCommand){
        return new StartEndCommand(
                ledCommand::schedule,
                restoreLEDs()::schedule);
    }

    public Command scheduleLEDcommand(Command ledCommand){
        return new InstantCommand(ledCommand::schedule);
    }

    public Command circleLEDs(Color[] colors) {
        return Commands.repeatingSequence(
                        setLEDsCommand(colors),
                        new WaitCommand(0.1),
                        new InstantCommand(() -> shiftLeds(colors)))
                .ignoringDisable(true);
    }

    public Command restoreLEDs() {
        return new InstantCommand(() ->
                CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(this)));
    }

    public enum LEDPattern {
        OFF,
        RAINBOW,
        SOLID,
        ALTERNATING_STATIC,
        ALTERNATING_MOVING,
        TRAIN_CIRCLE,
        TRAIN,
        RANDOM,
        EXPAND,
        BLINKING;
    }

    public Color getAllianceColor() {
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) return Color.Colors.TEAM_BLUE.color;
        else if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) return Color.Colors.RED.color;
        else return Color.Colors.WHITE.color;
    }

    private void setLedStrip(Color[] colors) {
        for (int i = 0; i < colors.length; i++) buffer.setLED(i, colors[i]);
        LedStrip.setData(buffer);
    }

    private void shiftTrain(Color[] colors, Color mainColor, Color trainColor, int trainLength, int offset) {
        tailIndex = findTailIndex(colors, trainColor);
        Arrays.fill(colors, mainColor);
        for (int i = 0; i < trainLength; i++) {
            colors[stayInBounds(i + tailIndex + offset, colors.length)] = trainColor;
        }
    }

    private int findHeadIndex(Color[] colors, Color trainColor) {
        for (int i = colors.length - 1; i >= 0; i--) {
            if (colors[i].equals(trainColor)) return i;
        }
        return -1;
    }

    private int findTailIndex(Color[] colors, Color trainColor) {
        for (int i = 0; i < colors.length; i++) {
            if (colors[i].equals(trainColor)) return i;
        }
        return -1;
    }

    private void shiftLeds(Color[] colors) {
        Color lastColor = colors[colors.length - 1];

        for (int i = colors.length - 2; i >= 0; i--) {
            colors[i + 1] = colors[i];
        }

        colors[0] = lastColor;
    }

    private int stayInBounds(int value, int length) {
        if (value >= length) return stayInBounds(value - length, length);
        if (value < 0) return stayInBounds(value + length, length);
        return value;
    }
}