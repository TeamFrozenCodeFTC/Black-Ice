package org.firstinspires.ftc.teamcode.blackice.core.commands;

public abstract class Command {
    public void start() {}
    public void update() {}
    public abstract boolean isFinished();
    
    
    
    public static Command singleAction(Runnable action) {
        return new Command() {
            @Override
            public void start() {
                action.run();
            }
            
            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }
    
    public double now() {
        return System.nanoTime() * 1e-9;
    }
    
    public Command untilAllFinish(Command other) {
        Command self = this;
        
        return new Command() {
            @Override
            public void start() {
                self.start();
                other.start();
            }
            
            @Override
            public void update() {
                if (!self.isFinished()) self.update();
                if (!other.isFinished()) other.update();
            }
            
            @Override
            public boolean isFinished() {
                return self.isFinished() && other.isFinished();
            }
        };
    }
    
    public Command untilEitherFinishes(Command other) {
        Command self = this;
        
        return new Command() {
            @Override
            public void start() {
                self.start();
                other.start();
            }
            
            @Override
            public void update() {
                if (!self.isFinished()) self.update();
                if (!other.isFinished()) other.update();
            }
            
            @Override
            public boolean isFinished() {
                return self.isFinished() || other.isFinished();
            }
        };
    }
    
    // Convenience for multiple commands
    public Command parallel(Command... others) {
        Command result = this;
        for (Command other : others) {
            result = result.untilAllFinish(other);
        }
        return result;
    }
    
    public static Command waitSeconds(double seconds) {
        return new Command() {
            private double start;
            
            @Override
            public void start() {
                start = now();
            }
            
            @Override
            public boolean isFinished() {
                return now() - start >= seconds;
            }
        };
    }
    
    public Command withTimeout(double seconds) {
        return this.untilEitherFinishes(waitSeconds(seconds));
    }
}
