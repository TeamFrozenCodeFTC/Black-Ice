package org.firstinspires.ftc.teamcode.blackice.core.commands;

public interface Command {
    default void start() {}
    default void update() {}
    boolean isFinished();
    
    static Command singleAction(Runnable action) {
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
    
    default double now() {
        return System.nanoTime() * 1e-9;
    }
    
    default Command untilAllFinish(Command other) {
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
    
    default Command untilEitherFinishes(Command other) {
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
    default Command parallel(Command... others) {
        Command result = this;
        for (Command other : others) {
            result = result.untilAllFinish(other);
        }
        return result;
    }
    
    static Command waitSeconds(double seconds) {
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
    
    default Command withTimeout(double seconds) {
        return this.untilEitherFinishes(waitSeconds(seconds));
    }
}
