package teamcode.CompOpModes.TeleopAndRobotFiles;

public final class PIDControllerJava {
    private long lastTimeStamp;
    private double lastError;
    private double errorSum;
    private double lowerBound;
    private double upperBound;
    private boolean isBounded;
    private boolean updated;
    private double kP;
    private double kI;
    private double kD;
    private double kG;

    public final void setBounds(double lowerBound, double upperBound) {
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
        this.isBounded = true;
    }

    public final double update(double error) {
        if (!this.updated) {
            this.lastTimeStamp = System.currentTimeMillis();
            this.lastError = error;
            this.updated = true;
            return 0.0D;
        } else {
            long now = System.currentTimeMillis();
            long dt = now - this.lastTimeStamp;
            this.errorSum += 0.5D * (error + this.lastError) * (double)dt;
            double derivative = (error - this.lastError) / (double)dt;
            this.lastError = error;
            this.lastTimeStamp = now;
            double p = error * this.kP;
            double d = derivative * this.kD;
            double i = this.errorSum * this.kI;
            double output = p + i + d + this.kG;
            double var10000;
            if (this.isBounded) {
                double var17 = this.lowerBound;
                double var19 = this.upperBound;
                var19 = Math.min(output, var19);
                var10000 = Math.max(var17, var19);
            } else {
                var10000 = output;
            }

            return var10000;
        }
    }

    public final void setConstants(double kP, double kI, double kD, double kG) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kG = kG;
    }

    public final double getKP() {
        return this.kP;
    }

    public final void setKP(double var1) {
        this.kP = var1;
    }

    public final double getKI() {
        return this.kI;
    }

    public final void setKI(double var1) {
        this.kI = var1;
    }

    public final double getKD() {
        return this.kD;
    }

    public final void setKD(double var1) {
        this.kD = var1;
    }

    public final double getKG() {
        return this.kG;
    }

    public final void setKG(double var1) {
        this.kG = var1;
    }

    public PIDControllerJava(double kP, double kI, double kD, double kG) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kG = kG;
    }
}
