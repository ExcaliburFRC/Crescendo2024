package frc.robot.util;

public class Color extends edu.wpi.first.wpilibj.util.Color {
    public Color(double red, double green, double blue) {
        super(green, red, blue);
    }

    public Color(int red, int green, int blue) {
        super(green, red, blue);
    }

    public Color(){
        super(0, 0, 0);
    }

    public static Color balance(Color color) {
        double v = Math.max(Math.max(color.red, color.green), color.blue);
        Color newColor = new Color(color.red, color.green / 2, color.blue / 4);
        double newV = Math.max(Math.max(newColor.red, newColor.green), newColor.blue);
        double ratio = v / newV;
        return new Color(newColor.red * ratio, newColor.green * ratio, newColor.blue * ratio);
    }

    public enum Colors {
        OFF(new Color(0, 0, 0)),

        TEAM_GOLD(new Color(255, 170, 0)),
        TEAM_BLUE(new Color(1, 30, 202)),

        RED(new Color(255, 0, 0)),
        GREEN(new Color(0, 255, 0)),
        BLUE(new Color(0, 0, 255)),

        WHITE(new Color(255, 255, 255)),

        YELLOW(new Color(255, 255, 0)),
        CYAN(new Color(0, 255, 255)),
        PINK(new Color(255, 0, 255)),

        ORANGE(new Color(255, 50, 0)),
        PURPLE(new Color(75, 0, 130));

        // TODO: add more colors

        public final Color color;

        Colors(Color color) {
            this.color = color;
        }
    }
}
