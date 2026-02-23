package org.firstinspires.ftc.teamcode.blackice;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LoggingTelemetry implements Telemetry {
    
    private final Telemetry telemetry;
    private final String tag;
    
    public LoggingTelemetry(Telemetry telemetry, String tag) {
        this.telemetry = telemetry;
        this.tag = tag;
    }
    
    // ----------------------------
    // addData
    // ----------------------------
    
    @Override
    public Item addData(String caption, Object value) {
        RobotLog.dd(tag, caption + ": " + value);
        return telemetry.addData(caption, value);
    }
    
    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        // We log lazily when update() happens
        return telemetry.addData(caption, valueProducer);
    }
    
    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return telemetry.addData(caption, format, valueProducer);
    }
    
    @Override
    public Item addData(String caption, String format, Object... args) {
        String message = String.format(format, args);
        RobotLog.dd(tag, caption + ": " + message);
        return telemetry.addData(caption, format, args);
    }
    
    // ----------------------------
    // Lines
    // ----------------------------
    
    @Override
    public Line addLine() {
        return telemetry.addLine();
    }
    
    @Override
    public Line addLine(String lineCaption) {
        RobotLog.dd(tag, lineCaption);
        return telemetry.addLine(lineCaption);
    }
    
    @Override
    public boolean removeLine(Line line) {
        return telemetry.removeLine(line);
    }
    
    // ----------------------------
    // Update
    // ----------------------------
    
    @Override
    public boolean update() {
        return telemetry.update();
    }
    
    // ----------------------------
    // Clear
    // ----------------------------
    
    @Override
    public void clear() {
        telemetry.clear();
    }
    
    @Override
    public void clearAll() {
        telemetry.clearAll();
    }
    
    // ----------------------------
    // Actions
    // ----------------------------
    
    @Override
    public Object addAction(Runnable action) {
        return telemetry.addAction(action);
    }
    
    @Override
    public boolean removeItem(Item item) {
        return telemetry.removeItem(item);
    }
    
    @Override
    public boolean removeAction(Object token) {
        return telemetry.removeAction(token);
    }
    
    // ----------------------------
    // Speech
    // ----------------------------
    
    @Override
    public void speak(String text) {
        RobotLog.dd(tag, "SPEAK: " + text);
        telemetry.speak(text);
    }
    
    @Override
    public void speak(String text, String languageCode, String countryCode) {
        RobotLog.dd(tag, "SPEAK: " + text);
        telemetry.speak(text, languageCode, countryCode);
    }
    
    // ----------------------------
    // Transmission
    // ----------------------------
    
    @Override
    public void setMsTransmissionInterval(int ms) {
        telemetry.setMsTransmissionInterval(ms);
    }
    
    @Override
    public int getMsTransmissionInterval() {
        return telemetry.getMsTransmissionInterval();
    }
    
    // ----------------------------
    // Formatting
    // ----------------------------
    
    @Override
    public boolean isAutoClear() {
        return telemetry.isAutoClear();
    }
    
    @Override
    public void setAutoClear(boolean autoClear) {
        telemetry.setAutoClear(autoClear);
    }
    
    @Override
    public String getItemSeparator() {
        return telemetry.getItemSeparator();
    }
    
    @Override
    public void setItemSeparator(String itemSeparator) {
        telemetry.setItemSeparator(itemSeparator);
    }
    
    @Override
    public String getCaptionValueSeparator() {
        return telemetry.getCaptionValueSeparator();
    }
    
    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        telemetry.setCaptionValueSeparator(captionValueSeparator);
    }
    
    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        telemetry.setDisplayFormat(displayFormat);
    }
    
    // ----------------------------
    // Log
    // ----------------------------
    
    @Override
    public Log log() {
        return telemetry.log();
    }
}
