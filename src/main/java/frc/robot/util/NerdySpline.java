package frc.robot.util;

public class NerdySpline {
    private double[] inputs;
    private double[] outputs;
    private double[] mM;

    
    /**
     * Spline values
     * based on https://gist.github.com/lecho/7627739
     * @param inputs
     * @param outputs
     */
    public NerdySpline(double[] inputs, double[] outputs) {
        if(inputs.length != outputs.length) throw new Error("Number of inputs and outputs don't match");
        if(inputs.length < 2) throw new Error("Can't have less than 2 points");

        this.inputs = inputs;
        this.outputs = outputs;
    }

    public void create() {
        double[] x = inputs;
        double[] y = outputs;

        if (x == null || y == null || x.length != y.length || x.length < 2) {
            throw new IllegalArgumentException("There must be at least two control "
                    + "points and the arrays must be of equal length.");
        }

        final int n = x.length;
        double[] d = new double[n - 1];
        double[] m = new double[n];

        // Compute slopes of secant lines between successive points.
        for (int i = 0; i < n - 1; i++) {
            double h = inputs[i + 1] - inputs[i];
            if (h <= 0f) {
                throw new IllegalArgumentException("The control points must all "
                        + "have strictly increasing X values.");
            }
            d[i] = (outputs[i + 1] - outputs[i]) / h;
        }

        // Initialize the tangents as the average of the secants.
        m[0] = d[0];
        for (int i = 1; i < n - 1; i++) {
            m[i] = (d[i - 1] + d[i]) * 0.5f;
        }
        m[n - 1] = d[n - 2];

        // Update the tangents to preserve monotonicity.
        for (int i = 0; i < n - 1; i++) {
            if (d[i] == 0f) { // successive Y values are equal
                m[i] = 0f;
                m[i + 1] = 0f;
            } else {
                double a = m[i] / d[i];
                double b = m[i + 1] / d[i];
                double h = Math.hypot(a, b);
                if (h > 9f) {
                    double t = 3f / h;
                    m[i] = t * a * d[i];
                    m[i + 1] = t * b * d[i];
                }
            }
        }
        
        inputs = x;
        outputs = y;
        mM = m;
    }

    public double getOutput(double input) {
        if(input < inputs[0] || input > inputs[inputs.length - 1]) return -1;

        // Find the index of the last point with smaller X
        int i = 0;
        while (input >= inputs[i + 1]) {
            i += 1;
            // Check if input is equal to any prerecorded values
            if (input == inputs[i]) {
                return outputs[i];
            }
        }

        // Perform cubic Hermite spline interpolation.
        double h = inputs[i + 1] - inputs[i];
        double t = (input - inputs[i]) / h;
        return (outputs[i] * (1 + 2 * t) + h * mM[i] * t) * (1 - t) * (1 - t)
                + (outputs[i + 1] * (3 - 2 * t) + h * mM[i + 1] * (t - 1)) * t * t;
    }
}
