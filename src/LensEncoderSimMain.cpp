#include <JuceHeader.h>
#include <array>
#include <cmath>
#include <cstdint>

namespace
{
constexpr int kDefaultOutputPort = 9100;
constexpr int kDefaultFreeDPort = 40000;
constexpr double kDefaultOutputRateHz = 60.0;

int signOf(double value)
{
    if (value > 0.0)
        return 1;
    if (value < 0.0)
        return -1;
    return 0;
}

void configureSlider(juce::Slider& slider, double min, double max, double step, double value)
{
    slider.setSliderStyle(juce::Slider::LinearHorizontal);
    slider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 90, 24);
    slider.setRange(min, max, step);
    slider.setValue(value, juce::dontSendNotification);
}

void writeInt24(std::array<std::uint8_t, 29>& packet, int offset, int value)
{
    const auto v = static_cast<std::uint32_t>(juce::jlimit(0, 0x7fffff, value));
    packet[static_cast<size_t>(offset)] = static_cast<std::uint8_t>((v >> 16) & 0xff);
    packet[static_cast<size_t>(offset + 1)] = static_cast<std::uint8_t>((v >> 8) & 0xff);
    packet[static_cast<size_t>(offset + 2)] = static_cast<std::uint8_t>(v & 0xff);
}

std::array<std::uint8_t, 29> buildFreeD1Packet(int cameraId, int zoom, int focus)
{
    std::array<std::uint8_t, 29> packet {};
    packet[0] = 0xd1;
    packet[1] = static_cast<std::uint8_t>(juce::jlimit(0, 255, cameraId));
    writeInt24(packet, 2, 0);   // pan
    writeInt24(packet, 5, 0);   // tilt
    writeInt24(packet, 8, 0);   // roll
    writeInt24(packet, 11, 0);  // x
    writeInt24(packet, 14, 0);  // y
    writeInt24(packet, 17, 0);  // z
    writeInt24(packet, 20, zoom);
    writeInt24(packet, 23, focus);
    packet[26] = 0;
    packet[27] = 0;

    std::uint32_t sum = 0;
    for (size_t i = 0; i < 28; ++i)
        sum += packet[i];
    packet[28] = static_cast<std::uint8_t>((256u - (sum & 0xffu)) & 0xffu);
    return packet;
}
} // namespace

class EncoderSimComponent final : public juce::Component,
                                  private juce::Timer
{
public:
    EncoderSimComponent()
        : connectButton("Apply Outputs"),
          centerZoomButton("Center")
    {
        titleLabel.setText("Broadcast Lens Encoder OSC Simulator", juce::dontSendNotification);
        titleLabel.setJustificationType(juce::Justification::centred);
        addAndMakeVisible(titleLabel);

        outHostLabel.setText("Out Host", juce::dontSendNotification);
        outPortLabel.setText("Out Port", juce::dontSendNotification);
        outAddressLabel.setText("Out Addr", juce::dontSendNotification);
        focusOutAddressLabel.setText("Focus Addr", juce::dontSendNotification);
        freeDEnableToggle.setButtonText("Send FreeD D1");
        freeDHostLabel.setText("FreeD Host", juce::dontSendNotification);
        freeDPortLabel.setText("FreeD Port", juce::dontSendNotification);
        freeDCameraIdLabel.setText("FreeD Cam ID", juce::dontSendNotification);

        outHostEditor.setText("127.0.0.1", juce::dontSendNotification);
        outPortEditor.setText(juce::String(kDefaultOutputPort), juce::dontSendNotification);
        outAddressEditor.setText("/lens/encoder", juce::dontSendNotification);
        focusOutAddressEditor.setText("/focus", juce::dontSendNotification);
        freeDEnableToggle.setToggleState(false, juce::dontSendNotification);
        freeDHostEditor.setText("127.0.0.1", juce::dontSendNotification);
        freeDPortEditor.setText(juce::String(kDefaultFreeDPort), juce::dontSendNotification);
        configureSlider(freeDCameraIdSlider, 0.0, 255.0, 1.0, 1.0);

        addAndMakeVisible(outHostLabel);
        addAndMakeVisible(outPortLabel);
        addAndMakeVisible(outAddressLabel);
        addAndMakeVisible(outHostEditor);
        addAndMakeVisible(outPortEditor);
        addAndMakeVisible(outAddressEditor);
        addAndMakeVisible(focusOutAddressLabel);
        addAndMakeVisible(focusOutAddressEditor);
        addAndMakeVisible(freeDEnableToggle);
        addAndMakeVisible(freeDHostLabel);
        addAndMakeVisible(freeDPortLabel);
        addAndMakeVisible(freeDCameraIdLabel);
        addAndMakeVisible(freeDHostEditor);
        addAndMakeVisible(freeDPortEditor);
        addAndMakeVisible(freeDCameraIdSlider);

        connectButton.onClick = [this] { applyConnection(); };
        addAndMakeVisible(connectButton);

        zoomLabel.setText("Zoom Position (0..1)", juce::dontSendNotification);
        configureSlider(zoomSlider, 0.0, 1.0, 0.001, 0.0);
        addAndMakeVisible(zoomLabel);
        addAndMakeVisible(zoomSlider);

        focusLabel.setText("Focus Position (0..1)", juce::dontSendNotification);
        configureSlider(focusSlider, 0.0, 1.0, 0.001, 0.5);
        addAndMakeVisible(focusLabel);
        addAndMakeVisible(focusSlider);

        focusEncMinLabel.setText("Focus Enc Min", juce::dontSendNotification);
        focusEncMaxLabel.setText("Focus Enc Max", juce::dontSendNotification);
        configureSlider(focusEncMinSlider, -200000.0, 200000.0, 1.0, 0.0);
        configureSlider(focusEncMaxSlider, -200000.0, 200000.0, 1.0, 1.0);
        addAndMakeVisible(focusEncMinLabel);
        addAndMakeVisible(focusEncMaxLabel);
        addAndMakeVisible(focusEncMinSlider);
        addAndMakeVisible(focusEncMaxSlider);

        centerZoomButton.onClick = [this]
        {
            zoomSlider.setValue(0.5, juce::sendNotificationSync);
        };
        addAndMakeVisible(centerZoomButton);

        outputRateLabel.setText("Output Hz", juce::dontSendNotification);
        configureSlider(outputRateSlider, 10.0, 240.0, 1.0, kDefaultOutputRateHz);
        outputRateSlider.onValueChange = [this] { restartTimerFromRate(); };
        addAndMakeVisible(outputRateLabel);
        addAndMakeVisible(outputRateSlider);

        minFocalLabel.setText("Min Focal mm", juce::dontSendNotification);
        maxFocalLabel.setText("Max Focal mm", juce::dontSendNotification);
        configureSlider(minFocalSlider, 3.0, 40.0, 0.1, 7.6);
        configureSlider(maxFocalSlider, 20.0, 250.0, 0.1, 152.0);
        addAndMakeVisible(minFocalLabel);
        addAndMakeVisible(maxFocalLabel);
        addAndMakeVisible(minFocalSlider);
        addAndMakeVisible(maxFocalSlider);

        gammaLabel.setText("Encoder Curve", juce::dontSendNotification);
        encoderMaxLabel.setText("Encoder Max", juce::dontSendNotification);
        configureSlider(gammaSlider, 0.5, 3.0, 0.01, 1.6);
        configureSlider(encoderMaxSlider, 1024.0, 65535.0, 1.0, 16383.0);
        addAndMakeVisible(gammaLabel);
        addAndMakeVisible(encoderMaxLabel);
        addAndMakeVisible(gammaSlider);
        addAndMakeVisible(encoderMaxSlider);

        backlashLabel.setText("Backlash Cnts", juce::dontSendNotification);
        noiseLabel.setText("Noise Cnts", juce::dontSendNotification);
        configureSlider(backlashSlider, 0.0, 200.0, 1.0, 8.0);
        configureSlider(noiseSlider, 0.0, 20.0, 1.0, 1.0);
        addAndMakeVisible(backlashLabel);
        addAndMakeVisible(noiseLabel);
        addAndMakeVisible(backlashSlider);
        addAndMakeVisible(noiseSlider);

        focalLabel.setText("Focal mm: 0.0", juce::dontSendNotification);
        zoomOutLabel.setText("Zoom position: 0.0", juce::dontSendNotification);
        focusOutLabel.setText("Focus position: 0.5 -> 0.5", juce::dontSendNotification);
        encoderLabel.setText("Encoder count: 0", juce::dontSendNotification);
        statusLabel.setText("OSC: disconnected | FreeD: disabled", juce::dontSendNotification);

        addAndMakeVisible(focalLabel);
        addAndMakeVisible(zoomOutLabel);
        addAndMakeVisible(focusOutLabel);
        addAndMakeVisible(encoderLabel);
        addAndMakeVisible(statusLabel);

        applyConnection();
        restartTimerFromRate();

        setSize(860, 680);
    }

    ~EncoderSimComponent() override
    {
        stopTimer();
        sender.disconnect();
        freeDSocket.reset();
    }

    void resized() override
    {
        auto area = getLocalBounds().reduced(12);
        titleLabel.setBounds(area.removeFromTop(30));
        area.removeFromTop(8);

        const int rowH = 28;
        const int labelW = 100;
        const int smallW = 95;
        const int mediumW = 160;
        const int buttonW = 130;
        const int gap = 8;

        auto row1 = area.removeFromTop(rowH);
        outHostLabel.setBounds(row1.removeFromLeft(labelW));
        outHostEditor.setBounds(row1.removeFromLeft(mediumW));
        row1.removeFromLeft(gap);
        outPortLabel.setBounds(row1.removeFromLeft(labelW));
        outPortEditor.setBounds(row1.removeFromLeft(smallW));
        row1.removeFromLeft(gap);
        outAddressLabel.setBounds(row1.removeFromLeft(labelW));
        outAddressEditor.setBounds(row1.removeFromLeft(mediumW));
        row1.removeFromLeft(gap);
        connectButton.setBounds(row1.removeFromLeft(buttonW));

        area.removeFromTop(8);

        auto row1b = area.removeFromTop(rowH);
        focusOutAddressLabel.setBounds(row1b.removeFromLeft(labelW));
        focusOutAddressEditor.setBounds(row1b.removeFromLeft(mediumW));

        area.removeFromTop(6);

        auto row1c = area.removeFromTop(rowH);
        freeDEnableToggle.setBounds(row1c.removeFromLeft(140));
        row1c.removeFromLeft(gap);
        freeDHostLabel.setBounds(row1c.removeFromLeft(labelW));
        freeDHostEditor.setBounds(row1c.removeFromLeft(mediumW));
        row1c.removeFromLeft(gap);
        freeDPortLabel.setBounds(row1c.removeFromLeft(labelW));
        freeDPortEditor.setBounds(row1c.removeFromLeft(smallW));
        row1c.removeFromLeft(gap);
        freeDCameraIdLabel.setBounds(row1c.removeFromLeft(100));
        freeDCameraIdSlider.setBounds(row1c);

        area.removeFromTop(6);

        auto row2 = area.removeFromTop(rowH);
        zoomLabel.setBounds(row2.removeFromLeft(labelW));
        centerZoomButton.setBounds(row2.removeFromRight(buttonW));
        row2.removeFromRight(gap);
        zoomSlider.setBounds(row2);

        area.removeFromTop(6);

        auto row2b = area.removeFromTop(rowH);
        focusLabel.setBounds(row2b.removeFromLeft(labelW));
        focusSlider.setBounds(row2b);

        area.removeFromTop(6);

        auto row2c = area.removeFromTop(rowH);
        focusEncMinLabel.setBounds(row2c.removeFromLeft(120));
        focusEncMinSlider.setBounds(row2c.removeFromLeft(280));
        row2c.removeFromLeft(gap);
        focusEncMaxLabel.setBounds(row2c.removeFromLeft(120));
        focusEncMaxSlider.setBounds(row2c);

        area.removeFromTop(6);

        auto row3 = area.removeFromTop(rowH);
        outputRateLabel.setBounds(row3.removeFromLeft(labelW));
        outputRateSlider.setBounds(row3);

        area.removeFromTop(10);

        auto row4 = area.removeFromTop(rowH);
        minFocalLabel.setBounds(row4.removeFromLeft(120));
        minFocalSlider.setBounds(row4.removeFromLeft(280));
        row4.removeFromLeft(gap);
        maxFocalLabel.setBounds(row4.removeFromLeft(120));
        maxFocalSlider.setBounds(row4);

        area.removeFromTop(6);

        auto row5 = area.removeFromTop(rowH);
        gammaLabel.setBounds(row5.removeFromLeft(120));
        gammaSlider.setBounds(row5.removeFromLeft(280));
        row5.removeFromLeft(gap);
        encoderMaxLabel.setBounds(row5.removeFromLeft(120));
        encoderMaxSlider.setBounds(row5);

        area.removeFromTop(6);

        auto row6 = area.removeFromTop(rowH);
        backlashLabel.setBounds(row6.removeFromLeft(120));
        backlashSlider.setBounds(row6.removeFromLeft(280));
        row6.removeFromLeft(gap);
        noiseLabel.setBounds(row6.removeFromLeft(120));
        noiseSlider.setBounds(row6);

        area.removeFromTop(12);

        focalLabel.setBounds(area.removeFromTop(24));
        zoomOutLabel.setBounds(area.removeFromTop(24));
        focusOutLabel.setBounds(area.removeFromTop(24));
        encoderLabel.setBounds(area.removeFromTop(24));
        statusLabel.setBounds(area.removeFromTop(24));
    }

private:
    void timerCallback() override
    {
        const auto minFocal = static_cast<float>(juce::jmin(minFocalSlider.getValue(), maxFocalSlider.getValue() - 0.1));
        const auto maxFocal = static_cast<float>(juce::jmax(maxFocalSlider.getValue(), minFocalSlider.getValue() + 0.1));

        const auto zoomPos01 = juce::jlimit(0.0f, 1.0f, static_cast<float>(zoomSlider.getValue()));
        const auto focusPos01 = juce::jlimit(0.0f, 1.0f, static_cast<float>(focusSlider.getValue()));
        const auto focusEncMin = static_cast<float>(focusEncMinSlider.getValue());
        auto focusEncMax = static_cast<float>(focusEncMaxSlider.getValue());
        if (focusEncMax <= focusEncMin)
            focusEncMax = focusEncMin + 1.0f;
        const auto focusOut = focusEncMin + focusPos01 * (focusEncMax - focusEncMin);
        currentFocalMm = minFocal + zoomPos01 * (maxFocal - minFocal);

        const auto gamma = static_cast<float>(gammaSlider.getValue());
        const auto curveNorm = std::pow(zoomPos01, juce::jmax(0.1f, gamma));

        const auto encoderMax = static_cast<int>(encoderMaxSlider.getValue());
        const auto idealEncoder = curveNorm * static_cast<float>(encoderMax);

        const auto direction = signOf(idealEncoder - encoderPositionCountsFloat);
        if (direction != 0 && direction != lastDirection)
            backlashRemainingCounts = static_cast<float>(backlashSlider.getValue());

        if (direction != 0)
        {
            auto deltaCounts = idealEncoder - encoderPositionCountsFloat;
            if (backlashRemainingCounts > 0.0f)
            {
                const auto consume = juce::jmin(std::abs(deltaCounts), backlashRemainingCounts);
                backlashRemainingCounts -= consume;
                deltaCounts = signOf(deltaCounts) * juce::jmax(0.0f, std::abs(deltaCounts) - consume);
            }
            encoderPositionCountsFloat += deltaCounts;
            lastDirection = direction;
        }

        const auto noiseRange = static_cast<int>(noiseSlider.getValue());
        const auto noise = noiseRange > 0 ? rng.nextInt(juce::Range<int>(-noiseRange, noiseRange + 1)) : 0;

        const auto encoderOut = juce::jlimit(0,
                                             encoderMax,
                                             static_cast<int>(std::lround(encoderPositionCountsFloat)) + noise);

        if (isConnected && outAddressEditor.getText().trim().isNotEmpty())
        {
            sender.send(juce::OSCMessage(outAddressEditor.getText().trim(),
                                         encoderOut,
                                         currentFocalMm,
                                         zoomPos01));
        }

        if (isConnected && focusOutAddressEditor.getText().trim().isNotEmpty())
            sender.send(juce::OSCMessage(focusOutAddressEditor.getText().trim(), focusOut));

        if (freeDConnected && freeDSocket != nullptr)
        {
            const auto freeDPort = freeDPortEditor.getText().trim().getIntValue();
            if (freeDPort > 0 && freeDPort <= 65535 && freeDHostEditor.getText().trim().isNotEmpty())
            {
                const auto cameraId = static_cast<int>(std::lround(freeDCameraIdSlider.getValue()));
                const auto freeDZoom = encoderOut;
                const auto freeDFocus = static_cast<int>(std::lround(focusOut));
                const auto packet = buildFreeD1Packet(cameraId, freeDZoom, freeDFocus);
                freeDSocket->write(freeDHostEditor.getText().trim(),
                                   freeDPort,
                                   packet.data(),
                                   static_cast<int>(packet.size()));
            }
        }

        focalLabel.setText("Focal mm: " + juce::String(currentFocalMm, 3), juce::dontSendNotification);
        zoomOutLabel.setText("Zoom position: " + juce::String(zoomPos01, 4), juce::dontSendNotification);
        focusOutLabel.setText(
            "Focus position: " + juce::String(focusPos01, 4) + " -> " + juce::String(focusOut, 4),
            juce::dontSendNotification);
        encoderLabel.setText("Encoder count: " + juce::String(encoderOut) + " / " + juce::String(encoderMax), juce::dontSendNotification);
    }

    void applyConnection()
    {
        sender.disconnect();
        isConnected = sender.connect(outHostEditor.getText().trim(), outPortEditor.getText().trim().getIntValue());
        freeDSocket.reset();
        freeDConnected = false;
        if (freeDEnableToggle.getToggleState())
        {
            const auto freeDPort = freeDPortEditor.getText().trim().getIntValue();
            if (freeDPort > 0 && freeDPort <= 65535 && freeDHostEditor.getText().trim().isNotEmpty())
            {
                freeDSocket = std::make_unique<juce::DatagramSocket>(false);
                freeDConnected = true;
            }
        }

        statusLabel.setText(
            "OSC: " + juce::String(isConnected ? "connected" : "failed")
                + " | FreeD: " + juce::String(freeDEnableToggle.getToggleState() ? (freeDConnected ? "enabled" : "failed") : "disabled"),
            juce::dontSendNotification);
    }

    void restartTimerFromRate()
    {
        const auto hz = juce::jlimit(1, 240, static_cast<int>(std::lround(outputRateSlider.getValue())));
        startTimerHz(hz);
    }

    juce::Label titleLabel;

    juce::Label outHostLabel;
    juce::Label outPortLabel;
    juce::Label outAddressLabel;
    juce::Label focusOutAddressLabel;
    juce::ToggleButton freeDEnableToggle;
    juce::Label freeDHostLabel;
    juce::Label freeDPortLabel;
    juce::Label freeDCameraIdLabel;
    juce::TextEditor outHostEditor;
    juce::TextEditor outPortEditor;
    juce::TextEditor outAddressEditor;
    juce::TextEditor focusOutAddressEditor;
    juce::TextEditor freeDHostEditor;
    juce::TextEditor freeDPortEditor;
    juce::Slider freeDCameraIdSlider;
    juce::TextButton connectButton;

    juce::Label zoomLabel;
    juce::Slider zoomSlider;
    juce::Label focusLabel;
    juce::Slider focusSlider;
    juce::Label focusEncMinLabel;
    juce::Slider focusEncMinSlider;
    juce::Label focusEncMaxLabel;
    juce::Slider focusEncMaxSlider;
    juce::TextButton centerZoomButton;

    juce::Label outputRateLabel;
    juce::Slider outputRateSlider;

    juce::Label minFocalLabel;
    juce::Slider minFocalSlider;
    juce::Label maxFocalLabel;
    juce::Slider maxFocalSlider;

    juce::Label gammaLabel;
    juce::Slider gammaSlider;
    juce::Label encoderMaxLabel;
    juce::Slider encoderMaxSlider;

    juce::Label backlashLabel;
    juce::Slider backlashSlider;
    juce::Label noiseLabel;
    juce::Slider noiseSlider;

    juce::Label focalLabel;
    juce::Label zoomOutLabel;
    juce::Label focusOutLabel;
    juce::Label encoderLabel;
    juce::Label statusLabel;

    juce::OSCSender sender;
    std::unique_ptr<juce::DatagramSocket> freeDSocket;
    juce::Random rng;

    float currentFocalMm = 0.0f;
    float encoderPositionCountsFloat = 0.0f;
    float backlashRemainingCounts = 0.0f;
    int lastDirection = 0;
    bool isConnected = false;
    bool freeDConnected = false;
};

class EncoderSimWindow final : public juce::DocumentWindow
{
public:
    explicit EncoderSimWindow(juce::String name)
        : juce::DocumentWindow(std::move(name),
                               juce::Desktop::getInstance().getDefaultLookAndFeel().findColour(juce::ResizableWindow::backgroundColourId),
                               juce::DocumentWindow::allButtons)
    {
        setUsingNativeTitleBar(true);
        setResizable(true, true);
        setResizeLimits(760, 620, 1600, 1200);
        setContentOwned(new EncoderSimComponent(), true);
        centreWithSize(getWidth(), getHeight());
        setVisible(true);
    }

    void closeButtonPressed() override
    {
        juce::JUCEApplication::getInstance()->systemRequestedQuit();
    }
};

class LensEncoderOscSimApp final : public juce::JUCEApplication
{
public:
    LensEncoderOscSimApp() = default;

    const juce::String getApplicationName() override { return "Lens Encoder OSC Sim"; }
    const juce::String getApplicationVersion() override { return "0.1.0"; }
    bool moreThanOneInstanceAllowed() override { return true; }

    void initialise(const juce::String&) override
    {
        window = std::make_unique<EncoderSimWindow>(getApplicationName());
    }

    void shutdown() override
    {
        window.reset();
    }

    void systemRequestedQuit() override
    {
        quit();
    }

private:
    std::unique_ptr<EncoderSimWindow> window;
};

START_JUCE_APPLICATION(LensEncoderOscSimApp)
