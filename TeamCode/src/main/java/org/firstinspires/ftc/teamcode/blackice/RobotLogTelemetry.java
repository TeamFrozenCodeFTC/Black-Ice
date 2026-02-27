package org.firstinspires.ftc.teamcode.blackice;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotLogTelemetry implements Telemetry {
    private final String tag;
    
    public RobotLogTelemetry(String tag) {
        this.tag = tag;
    }
    
    // ----------------------------
    // addData
    // ----------------------------
    
    @Override
    public Item addData(String caption, Object value) {
        RobotLog.dd(tag, "%s: %s", caption, value);
        return null; // do nothing
    }
    
    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        // nothing
        return null;
    }
    
    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        // nothing
        return null;
    }
    
    @Override
    public Item addData(String caption, String format, Object... args) {
        String message = String.format(format, args);
        RobotLog.dd(tag, "%s: %s", caption, message);
        return null;
    }
    
    // ----------------------------
    // Lines
    // ----------------------------
    
    @Override
    public Line addLine() {
        return null;
    }
    
    @Override
    public Line addLine(String lineCaption) {
        RobotLog.dd(tag, lineCaption);
        return null;
    }
    
    @Override
    public boolean removeLine(Line line) {
        return false;
    }
    
    // ----------------------------
    // Update / Clear
    // ----------------------------
    
    @Override
    public boolean update() {
        return true; // do nothing but indicate success
    }
    
    @Override
    public void clear() {}
    @Override
    public void clearAll() {}
    
    // ----------------------------
    // Actions
    // ----------------------------
    
    @Override
    public Object addAction(Runnable action) { return null; }
    @Override
    public boolean removeItem(Item item) { return false; }
    @Override
    public boolean removeAction(Object token) { return false; }
    
    // ----------------------------
    // Speech
    // ----------------------------
    
    @Override
    public void speak(String text) {
        RobotLog.dd(tag, "SPEAK: %s", text);
    }
    
    @Override
    public void speak(String text, String languageCode, String countryCode) {
        RobotLog.dd(tag, "SPEAK: %s [%s-%s]", text, languageCode, countryCode);
    }
    
    // ----------------------------
    // Transmission / Formatting
    // ----------------------------
    
    @Override public void setMsTransmissionInterval(int ms) {}
    @Override public int getMsTransmissionInterval() { return 0; }
    @Override public boolean isAutoClear() { return false; }
    @Override public void setAutoClear(boolean autoClear) {}
    @Override public String getItemSeparator() { return null; }
    @Override public void setItemSeparator(String itemSeparator) {}
    @Override public String getCaptionValueSeparator() { return null; }
    @Override public void setCaptionValueSeparator(String captionValueSeparator) {}
    @Override public void setDisplayFormat(DisplayFormat displayFormat) {}
    
    // ----------------------------
    // Log
    // ----------------------------
    @Override
    public Log log() { return null; }
}
