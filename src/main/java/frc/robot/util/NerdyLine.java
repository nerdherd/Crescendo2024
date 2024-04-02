package frc.robot.util;

public class NerdyLine {
    private double[] inputs;
    private double[] outputs;
    
    /**
     * Linear Interpolation across several points
     * Note: inputs must ordered.
     * @param inputs
     * @param outputs
     */
    public NerdyLine(double[] inputs, double[] outputs) {
        if(inputs.length != outputs.length) throw new Error("Number of inputs and outputs don't match");
        if(inputs.length < 2) throw new Error("Can't have less than 2 points");

        this.inputs = inputs;
        this.outputs = outputs;
    }

    public double getOutput(double input) {
        if (input < inputs[0] || input > inputs[inputs.length - 1]) {
            return -1;
        }

        int index = 0;
        while (index < inputs.length - 1) {
            if (input >= inputs[index] && input <= inputs[index + 1]) {
                return NerdyMath.map(input, inputs[index], inputs[index + 1], 
                                            outputs[index], outputs[index + 1]);
            }
            index++;
        }

        return -1;
    }
}
