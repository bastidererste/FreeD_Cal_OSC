#include <JuceHeader.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#if LININTERPOL_HAVE_ALGLIB
#include "interpolation.h"
#endif

namespace
{
constexpr int kDefaultInputPort = 9000;
constexpr int kDefaultOutputPort = 9001;
constexpr int kDefaultFreeDPort = 40000;
constexpr double kDefaultSmoothingMs = 120.0;
constexpr double kDefaultOutputRateHz = 60.0;
constexpr int kMinLutEntryCount = 4;
constexpr int kMinSurfaceAxisCount = 2;
constexpr float kNewLutDefaultOutputAtMin = 150.0f;
constexpr float kNewLutDefaultOutputAtMax = 1.0f;
constexpr const char* kSettingsRootTag = "LinInterpolSettings";
constexpr const char* kPresetTag = "Preset";

struct ParsedLutData
{
    std::vector<float> inputValues;
    std::vector<float> outputValues;
    bool hasExplicitInput = false;
};

struct ParsedSurfaceData
{
    std::vector<float> lensAxis;
    std::vector<float> focusAxis;
    std::vector<float> values; // row-major: lens index first, focus index second
};

struct ParsedFreeD1Packet
{
    int cameraId = 0;
    int zoom = 0;
    int focus = 0;
};

int parseFreeDInt24LikeCSharp(const std::uint8_t* bytes3)
{
    int value = 1;
    value <<= 8;
    value |= bytes3[0];
    value <<= 8;
    value |= bytes3[1];
    value <<= 8;
    value |= bytes3[2];
    const bool signBitOn = (value & 0x00800000) != 0;
    if (!signBitOn)
        value &= 0x00ffffff;
    return value;
}

bool parseFreeD1Packet(const std::uint8_t* data, int size, ParsedFreeD1Packet& out)
{
    constexpr int kMinPacketSize = 29; // 0xd1 + camera + payload + checksum
    if (data == nullptr || size < kMinPacketSize)
        return false;
    if (data[0] != 0xd1)
        return false;

    out.cameraId = static_cast<int>(data[1]);
    out.zoom = parseFreeDInt24LikeCSharp(data + 20);
    out.focus = parseFreeDInt24LikeCSharp(data + 23);
    return true;
}

float parseFirstOscValue(const juce::OSCMessage& message, bool& ok)
{
    ok = false;
    if (message.size() <= 0)
        return 0.0f;

    const auto& arg = message[0];
    if (arg.isFloat32())
    {
        ok = true;
        return arg.getFloat32();
    }

    if (arg.isInt32())
    {
        ok = true;
        return static_cast<float>(arg.getInt32());
    }

    if (arg.isString())
    {
        const auto text = arg.getString().trim();
        if (text.containsAnyOf("0123456789") && text.containsOnly("0123456789+-.eE"))
        {
            ok = true;
            return text.getFloatValue();
        }
    }

    return 0.0f;
}

bool parseNumericToken(const juce::String& token, float& valueOut)
{
    const auto t = token.trim();
    if (!(t.containsAnyOf("0123456789") && t.containsOnly("0123456789+-.eE")))
        return false;

    valueOut = t.getFloatValue();
    return true;
}

bool parseLutText(const juce::String& text, ParsedLutData& dataOut, juce::String& error)
{
    dataOut = {};

    juce::StringArray lines;
    lines.addLines(text);

    // Pair mode: one "input,output" pair per line.
    if (lines.size() >= kMinLutEntryCount)
    {
        bool allLinesArePairs = true;
        int nonEmptyLines = 0;
        for (const auto& line : lines)
        {
            const auto trimmed = line.trim();
            if (trimmed.isEmpty())
                continue;

            ++nonEmptyLines;
            juce::StringArray cols;
            cols.addTokens(trimmed, ",; \t", "");
            cols.trim();
            cols.removeEmptyStrings();
            if (cols.size() != 2)
            {
                allLinesArePairs = false;
                break;
            }
        }

        if (allLinesArePairs && nonEmptyLines >= kMinLutEntryCount)
        {
            dataOut.hasExplicitInput = true;
            dataOut.inputValues.reserve(static_cast<size_t>(nonEmptyLines));
            dataOut.outputValues.reserve(static_cast<size_t>(nonEmptyLines));

            int lineNo = 0;
            for (const auto& line : lines)
            {
                ++lineNo;
                const auto trimmed = line.trim();
                if (trimmed.isEmpty())
                    continue;

                juce::StringArray cols;
                cols.addTokens(trimmed, ",; \t", "");
                cols.trim();
                cols.removeEmptyStrings();

                float inValue = 0.0f;
                float outValue = 0.0f;
                if (!parseNumericToken(cols[0], inValue) || !parseNumericToken(cols[1], outValue))
                {
                    error = "Invalid input/output pair at line " + juce::String(lineNo);
                    dataOut = {};
                    return false;
                }

                dataOut.inputValues.push_back(inValue);
                dataOut.outputValues.push_back(outValue);
            }

            return true;
        }
    }

    // Fallback mode: output-only values (input axis comes from Input Min/Max).
    juce::StringArray tokens;
    tokens.addTokens(text, ",; \t\r\n", "");
    tokens.trim();
    tokens.removeEmptyStrings();

    if (tokens.size() < kMinLutEntryCount)
    {
        error = "Expected at least " + juce::String(kMinLutEntryCount) + " values, got " + juce::String(tokens.size());
        return false;
    }

    dataOut.hasExplicitInput = false;
    dataOut.outputValues.clear();
    dataOut.outputValues.reserve(static_cast<size_t>(tokens.size()));
    for (int i = 0; i < tokens.size(); ++i)
    {
        float value = 0.0f;
        if (!parseNumericToken(tokens[i], value))
        {
            error = "Invalid numeric token at index " + juce::String(i) + ": " + tokens[i].trim();
            dataOut = {};
            return false;
        }
        dataOut.outputValues.push_back(value);
    }

    return true;
}

bool parseSurfaceText(const juce::String& text, ParsedSurfaceData& dataOut, juce::String& error)
{
    dataOut = {};

    juce::StringArray lines;
    lines.addLines(text);

    struct Row
    {
        float lens = 0.0f;
        float focus = 0.0f;
        float fov = 0.0f;
    };

    std::vector<Row> rows;
    rows.reserve(static_cast<size_t>(lines.size()));

    int lineNo = 0;
    for (const auto& line : lines)
    {
        ++lineNo;
        const auto trimmed = line.trim();
        if (trimmed.isEmpty())
            continue;

        juce::StringArray cols;
        cols.addTokens(trimmed, ",; \t", "");
        cols.trim();
        cols.removeEmptyStrings();
        if (cols.size() != 3)
        {
            error = "Surface format: expected lens,focus,fov at line " + juce::String(lineNo);
            return false;
        }

        Row row;
        if (!parseNumericToken(cols[0], row.lens) || !parseNumericToken(cols[1], row.focus) || !parseNumericToken(cols[2], row.fov))
        {
            error = "Surface format: invalid numeric token at line " + juce::String(lineNo);
            return false;
        }
        rows.push_back(row);
    }

    if (rows.empty())
    {
        error = "Surface file is empty";
        return false;
    }

    dataOut.lensAxis.reserve(rows.size());
    dataOut.focusAxis.reserve(rows.size());
    for (const auto& r : rows)
    {
        dataOut.lensAxis.push_back(r.lens);
        dataOut.focusAxis.push_back(r.focus);
    }

    std::sort(dataOut.lensAxis.begin(), dataOut.lensAxis.end());
    dataOut.lensAxis.erase(std::unique(dataOut.lensAxis.begin(), dataOut.lensAxis.end()), dataOut.lensAxis.end());
    std::sort(dataOut.focusAxis.begin(), dataOut.focusAxis.end());
    dataOut.focusAxis.erase(std::unique(dataOut.focusAxis.begin(), dataOut.focusAxis.end()), dataOut.focusAxis.end());

    if (dataOut.lensAxis.size() < static_cast<size_t>(kMinSurfaceAxisCount)
        || dataOut.focusAxis.size() < static_cast<size_t>(kMinSurfaceAxisCount))
    {
        error = "Surface needs at least " + juce::String(kMinSurfaceAxisCount) + " unique lens and focus values";
        return false;
    }

    const auto lensCount = dataOut.lensAxis.size();
    const auto focusCount = dataOut.focusAxis.size();
    dataOut.values.assign(lensCount * focusCount, std::numeric_limits<float>::quiet_NaN());

    constexpr float kEpsilon = 1.0e-6f;
    for (const auto& r : rows)
    {
        const auto lensIt = std::lower_bound(dataOut.lensAxis.begin(), dataOut.lensAxis.end(), r.lens);
        const auto focusIt = std::lower_bound(dataOut.focusAxis.begin(), dataOut.focusAxis.end(), r.focus);
        if (lensIt == dataOut.lensAxis.end() || focusIt == dataOut.focusAxis.end())
            continue;
        if (std::abs(*lensIt - r.lens) > kEpsilon || std::abs(*focusIt - r.focus) > kEpsilon)
            continue;

        const auto lensIdx = static_cast<size_t>(std::distance(dataOut.lensAxis.begin(), lensIt));
        const auto focusIdx = static_cast<size_t>(std::distance(dataOut.focusAxis.begin(), focusIt));
        dataOut.values[lensIdx * focusCount + focusIdx] = r.fov;
    }

    for (size_t i = 0; i < lensCount; ++i)
    {
        for (size_t j = 0; j < focusCount; ++j)
        {
            const auto v = dataOut.values[i * focusCount + j];
            if (!std::isfinite(v))
            {
                error = "Surface grid is incomplete. Missing lens=" + juce::String(dataOut.lensAxis[i], 6)
                    + ", focus=" + juce::String(dataOut.focusAxis[j], 6);
                return false;
            }
        }
    }

    return true;
}
} // namespace

class HistoryPlot final : public juce::Component
{
public:
    HistoryPlot()
    {
        inputHistory.resize(static_cast<size_t>(capacity), 0.0f);
        outputHistory.resize(static_cast<size_t>(capacity), 0.0f);
    }

    void pushSample(float inValue, float outValue)
    {
        inputHistory[static_cast<size_t>(writeIndex)] = inValue;
        outputHistory[static_cast<size_t>(writeIndex)] = outValue;

        writeIndex = (writeIndex + 1) % capacity;
        sampleCount = juce::jmin(sampleCount + 1, capacity);
        repaint();
    }

    void setYRange(float minValue, float maxValue)
    {
        yMin = minValue;
        yMax = maxValue;
    }

    void paint(juce::Graphics& g) override
    {
        g.fillAll(juce::Colours::black);

        auto area = getLocalBounds().reduced(6).toFloat();
        g.setColour(juce::Colours::darkgrey);
        g.drawRect(area);

        if (sampleCount < 2)
            return;

        auto minVal = yMin;
        auto maxVal = yMax;
        if (maxVal <= minVal)
        {
            minVal = 0.0f;
            maxVal = 1.0f;
        }

        juce::Path inPath;
        juce::Path outPath;

        for (int i = 0; i < sampleCount; ++i)
        {
            const auto idx = historyIndexFromOldest(i);
            const auto x = juce::jmap(static_cast<float>(i), 0.0f, static_cast<float>(sampleCount - 1), area.getX(), area.getRight());
            const auto inY = juce::jmap(inputHistory[static_cast<size_t>(idx)], maxVal, minVal, area.getY(), area.getBottom());
            const auto outY = juce::jmap(outputHistory[static_cast<size_t>(idx)], maxVal, minVal, area.getY(), area.getBottom());

            if (i == 0)
            {
                inPath.startNewSubPath(x, inY);
                outPath.startNewSubPath(x, outY);
            }
            else
            {
                inPath.lineTo(x, inY);
                outPath.lineTo(x, outY);
            }
        }

        g.setColour(juce::Colours::deepskyblue);
        g.strokePath(inPath, juce::PathStrokeType(2.0f));

        g.setColour(juce::Colours::orange);
        g.strokePath(outPath, juce::PathStrokeType(2.0f));

        g.setColour(juce::Colours::white.withAlpha(0.85f));
        g.drawText("In", area.removeFromTop(16).toNearestInt(), juce::Justification::left);
        g.setColour(juce::Colours::deepskyblue);
        g.fillEllipse(area.getX(), area.getY() + 2.0f, 8.0f, 8.0f);

        area.removeFromTop(2.0f);
        g.setColour(juce::Colours::white.withAlpha(0.85f));
        g.drawText("Out", area.removeFromTop(16).toNearestInt(), juce::Justification::left);
        g.setColour(juce::Colours::orange);
        g.fillEllipse(area.getX(), area.getY() + 2.0f, 8.0f, 8.0f);
    }

private:
    int historyIndexFromOldest(int ageIndex) const
    {
        const auto oldest = (writeIndex - sampleCount + capacity) % capacity;
        return (oldest + ageIndex) % capacity;
    }

    static constexpr int capacity = 600;
    std::vector<float> inputHistory;
    std::vector<float> outputHistory;
    int writeIndex = 0;
    int sampleCount = 0;
    float yMin = 0.0f;
    float yMax = 1.0f;
};

class LutCartesianPlot final : public juce::Component
{
public:
    std::function<void(size_t, float)> onLutPointDragged;
    std::function<void()> onLutDragStarted;
    std::function<void()> onLutDragEnded;
    std::function<void(size_t)> onLutPointClicked;
    std::function<void(size_t, size_t, float)> onSurfacePointDragged;
    std::function<void()> onSurfaceDragStarted;
    std::function<void()> onSurfaceDragEnded;
    std::function<void(size_t, size_t)> onSurfacePointClicked;
    std::function<void(float)> onZoomChanged;

    void setInputRange(float minValue, float maxValue)
    {
        xMin = minValue;
        xMax = maxValue;
        if (lutOutputValues.empty())
            updateStaticYRange();
    }

    void setLutValues(const std::vector<float>& outputValues, const std::vector<float>& inputValues, bool keepCurrentYRange = false)
    {
        lutOutputValues = outputValues;
        lutInputValues = inputValues;
        if (!keepCurrentYRange)
            updateStaticYRange();
        repaint();
    }

    void setCubicCurve(const std::vector<float>& xValues, const std::vector<float>& yValues)
    {
        cubicCurveX = xValues;
        cubicCurveY = yValues;
        repaint();
    }

    void setSurfaceValues(const std::vector<float>& lensValues,
                          const std::vector<float>& focusValues,
                          const std::vector<float>& gridValues)
    {
        surfaceLensAxis = lensValues;
        surfaceFocusAxis = focusValues;
        surfaceValuesGrid = gridValues;
        updateStaticYRange();
        repaint();
    }

    void setPoints(float input, float lutSample, float cubicOutput)
    {
        inputValue = input;
        lutSampleValue = lutSample;
        cubicOutputValue = cubicOutput;
        repaint();
    }

    void setSelectedPointIndex(int index)
    {
        selectedPointIndex = index;
        repaint();
    }

    void setSelectedSurfacePoint(int lensIndex, int focusIndex)
    {
        selectedSurfaceLensIndex = lensIndex;
        selectedSurfaceFocusIndex = focusIndex;
        repaint();
    }

    void setZoomFactor(float factor)
    {
        zoomFactor = juce::jlimit(1.0f, 40.0f, factor);
        if (onZoomChanged)
            onZoomChanged(zoomFactor);
        repaint();
    }

    float getZoomFactor() const
    {
        return zoomFactor;
    }

    void mouseWheelMove(const juce::MouseEvent& event, const juce::MouseWheelDetails& wheel) override
    {
        auto area = getLocalBounds().reduced(6).toFloat();
        const auto plot = area.reduced(8.0f, 8.0f);
        if (plot.isEmpty())
            return;

        const auto wheelDelta = (std::abs(wheel.deltaY) > std::abs(wheel.deltaX)) ? wheel.deltaY : wheel.deltaX;
        if (std::abs(wheelDelta) <= 1.0e-6f)
            return;

        float currentMinX = 0.0f;
        float currentMaxX = 1.0f;
        float currentMinY = 0.0f;
        float currentMaxY = 1.0f;
        getViewRanges(currentMinX, currentMaxX, currentMinY, currentMaxY);

        const auto u = juce::jlimit(0.0f, 1.0f, (event.position.x - plot.getX()) / juce::jmax(1.0f, plot.getWidth()));
        const auto v = juce::jlimit(0.0f, 1.0f, (event.position.y - plot.getY()) / juce::jmax(1.0f, plot.getHeight()));
        const auto dataX = currentMinX + u * (currentMaxX - currentMinX);
        const auto dataY = currentMaxY - v * (currentMaxY - currentMinY);

        const auto oldZoom = juce::jmax(1.0f, zoomFactor);
        const auto zoomStep = std::exp(wheelDelta * 0.45f);
        const auto newZoom = juce::jlimit(1.0f, 40.0f, oldZoom * zoomStep);
        if (std::abs(newZoom - oldZoom) <= 1.0e-6f)
            return;

        auto fullMinX = xMin;
        auto fullMaxX = xMax;
        if (fullMaxX <= fullMinX)
        {
            fullMinX = 0.0f;
            fullMaxX = 1.0f;
        }

        auto fullMinY = yMin;
        auto fullMaxY = yMax;
        if (fullMaxY <= fullMinY)
        {
            fullMinY -= 1.0f;
            fullMaxY += 1.0f;
        }

        const auto newViewW = (fullMaxX - fullMinX) / newZoom;
        const auto newViewH = (fullMaxY - fullMinY) / newZoom;
        viewCenterX = dataX + (0.5f - u) * newViewW;
        viewCenterY = dataY + (v - 0.5f) * newViewH;
        viewCenterInitialized = true;

        setZoomFactor(newZoom);
    }

    void mouseDown(const juce::MouseEvent& event) override
    {
        draggedPointIndex = -1;
        mouseDownPointIndex = -1;
        dragMovedSinceMouseDown = false;
        dragSessionActive = false;
        panSessionActive = false;
        draggedSurfaceLensIndex = -1;
        draggedSurfaceFocusIndex = -1;
        mouseDownSurfaceLensIndex = -1;
        mouseDownSurfaceFocusIndex = -1;
        surfaceDragSessionActive = false;
        mouseDownPosition = event.position;
        if (lutOutputValues.empty() && !hasCompleteSurfaceData())
            return;

        auto area = getLocalBounds().reduced(6).toFloat();
        const auto plot = area.reduced(8.0f, 8.0f);
        if (plot.isEmpty())
            return;

        float minX = 0.0f;
        float maxX = 1.0f;
        float minY = 0.0f;
        float maxY = 1.0f;
        getViewRanges(minX, maxX, minY, maxY);

        constexpr float kPickRadiusPx = 14.0f;

        if (hasCompleteSurfaceData())
        {
            auto bestDistSq = kPickRadiusPx * kPickRadiusPx;
            const auto lensCount = static_cast<int>(surfaceLensAxis.size());
            const auto focusCount = static_cast<int>(surfaceFocusAxis.size());
            for (int i = 0; i < lensCount; ++i)
            {
                for (int j = 0; j < focusCount; ++j)
                {
                    const auto x = juce::jmap(surfaceLensAxis[static_cast<size_t>(i)], minX, maxX, plot.getX(), plot.getRight());
                    const auto y = juce::jmap(surfaceValuesGrid[static_cast<size_t>(i * focusCount + j)], maxY, minY, plot.getY(), plot.getBottom());
                    const auto distSq = event.position.getDistanceSquaredFrom({ x, y });
                    if (distSq <= bestDistSq)
                    {
                        bestDistSq = distSq;
                        draggedSurfaceLensIndex = i;
                        draggedSurfaceFocusIndex = j;
                    }
                }
            }

            if (draggedSurfaceLensIndex >= 0)
            {
                mouseDownSurfaceLensIndex = draggedSurfaceLensIndex;
                mouseDownSurfaceFocusIndex = draggedSurfaceFocusIndex;
                return;
            }
        }

        const bool hasExplicitX = (!lutInputValues.empty() && lutInputValues.size() == lutOutputValues.size());
        auto bestDistSq = kPickRadiusPx * kPickRadiusPx;
        for (int i = 0; i < static_cast<int>(lutOutputValues.size()); ++i)
        {
            const auto t = lutOutputValues.size() == 1 ? 0.0f : static_cast<float>(i) / static_cast<float>(lutOutputValues.size() - 1);
            const auto xValue = hasExplicitX ? lutInputValues[static_cast<size_t>(i)] : (minX + t * (maxX - minX));
            const auto yValue = lutOutputValues[static_cast<size_t>(i)];
            const auto x = juce::jmap(xValue, minX, maxX, plot.getX(), plot.getRight());
            const auto y = juce::jmap(yValue, maxY, minY, plot.getY(), plot.getBottom());
            const auto distSq = event.position.getDistanceSquaredFrom({ x, y });
            if (distSq <= bestDistSq)
            {
                bestDistSq = distSq;
                draggedPointIndex = i;
            }
        }

        mouseDownPointIndex = draggedPointIndex;
        if (draggedPointIndex < 0 && draggedSurfaceLensIndex < 0)
        {
            panSessionActive = true;
            panStartMousePosition = event.position;
            panStartZoomFactor = juce::jmax(1.0f, zoomFactor);
            float panMinX = 0.0f;
            float panMaxX = 1.0f;
            float panMinY = 0.0f;
            float panMaxY = 1.0f;
            getViewRanges(panMinX, panMaxX, panMinY, panMaxY);
            panStartCenterX = 0.5f * (panMinX + panMaxX);
            panStartCenterY = 0.5f * (panMinY + panMaxY);
            panStartViewWidth = panMaxX - panMinX;
            panStartViewHeight = panMaxY - panMinY;
        }
    }

    void mouseDrag(const juce::MouseEvent& event) override
    {
        if (draggedSurfaceLensIndex >= 0 && draggedSurfaceFocusIndex >= 0)
        {
            if (!event.mods.isShiftDown())
                return;

            if (!surfaceDragSessionActive)
            {
                surfaceDragSessionActive = true;
                if (onSurfaceDragStarted)
                    onSurfaceDragStarted();
            }

            if (!dragMovedSinceMouseDown && event.position.getDistanceFrom(mouseDownPosition) > 2.0f)
                dragMovedSinceMouseDown = true;

            auto area = getLocalBounds().reduced(6).toFloat();
            const auto plot = area.reduced(8.0f, 8.0f);
            if (plot.isEmpty())
                return;

            float dummyMinX = 0.0f;
            float dummyMaxX = 1.0f;
            float minY = 0.0f;
            float maxY = 1.0f;
            getViewRanges(dummyMinX, dummyMaxX, minY, maxY);

            const auto clampedY = juce::jlimit(plot.getY(), plot.getBottom(), event.position.y);
            const auto newValue = juce::jmap(clampedY, plot.getY(), plot.getBottom(), maxY, minY);
            const auto focusCount = static_cast<int>(surfaceFocusAxis.size());
            auto& editedValue = surfaceValuesGrid[static_cast<size_t>(draggedSurfaceLensIndex * focusCount + draggedSurfaceFocusIndex)];
            if (std::abs(newValue - editedValue) <= 1.0e-6f)
                return;

            editedValue = newValue;
            if (onSurfacePointDragged)
            {
                onSurfacePointDragged(
                    static_cast<size_t>(draggedSurfaceLensIndex),
                    static_cast<size_t>(draggedSurfaceFocusIndex),
                    newValue);
            }
            repaint();
            return;
        }

        if (draggedPointIndex >= 0 && draggedPointIndex < static_cast<int>(lutOutputValues.size()))
        {
            if (!event.mods.isShiftDown())
                return;

            if (!dragSessionActive)
            {
                dragSessionActive = true;
                if (onLutDragStarted)
                    onLutDragStarted();
            }

            if (!dragMovedSinceMouseDown && event.position.getDistanceFrom(mouseDownPosition) > 2.0f)
                dragMovedSinceMouseDown = true;

            auto area = getLocalBounds().reduced(6).toFloat();
            const auto plot = area.reduced(8.0f, 8.0f);
            if (plot.isEmpty())
                return;

            float dummyMinX = 0.0f;
            float dummyMaxX = 1.0f;
            float minY = 0.0f;
            float maxY = 1.0f;
            getViewRanges(dummyMinX, dummyMaxX, minY, maxY);

            const auto clampedY = juce::jlimit(plot.getY(), plot.getBottom(), event.position.y);
            const auto newValue = juce::jmap(clampedY, plot.getY(), plot.getBottom(), maxY, minY);
            auto& editedValue = lutOutputValues[static_cast<size_t>(draggedPointIndex)];
            if (std::abs(newValue - editedValue) <= 1.0e-6f)
                return;

            editedValue = newValue;
            if (onLutPointDragged)
                onLutPointDragged(static_cast<size_t>(draggedPointIndex), newValue);

            repaint();
            return;
        }

        if (!panSessionActive)
            return;

        auto area = getLocalBounds().reduced(6).toFloat();
        const auto plot = area.reduced(8.0f, 8.0f);
        if (plot.isEmpty())
            return;

        if (panStartZoomFactor <= 1.0001f)
            return;

        if (!dragMovedSinceMouseDown && event.position.getDistanceFrom(mouseDownPosition) > 2.0f)
            dragMovedSinceMouseDown = true;

        auto fullMinX = xMin;
        auto fullMaxX = xMax;
        if (fullMaxX <= fullMinX)
        {
            fullMinX = 0.0f;
            fullMaxX = 1.0f;
        }

        auto fullMinY = yMin;
        auto fullMaxY = yMax;
        if (fullMaxY <= fullMinY)
        {
            fullMinY -= 1.0f;
            fullMaxY += 1.0f;
        }

        const auto dxPx = event.position.x - panStartMousePosition.x;
        const auto dyPx = event.position.y - panStartMousePosition.y;
        const auto dataPerPxX = panStartViewWidth / juce::jmax(1.0f, plot.getWidth());
        const auto dataPerPxY = panStartViewHeight / juce::jmax(1.0f, plot.getHeight());

        auto centerX = panStartCenterX - dxPx * dataPerPxX;
        auto centerY = panStartCenterY + dyPx * dataPerPxY;

        const auto halfViewX = 0.5f * panStartViewWidth;
        const auto halfViewY = 0.5f * panStartViewHeight;
        const auto minCenterX = fullMinX + halfViewX;
        const auto maxCenterX = fullMaxX - halfViewX;
        const auto minCenterY = fullMinY + halfViewY;
        const auto maxCenterY = fullMaxY - halfViewY;
        if (maxCenterX > minCenterX)
            centerX = juce::jlimit(minCenterX, maxCenterX, centerX);
        else
            centerX = 0.5f * (fullMinX + fullMaxX);
        if (maxCenterY > minCenterY)
            centerY = juce::jlimit(minCenterY, maxCenterY, centerY);
        else
            centerY = 0.5f * (fullMinY + fullMaxY);

        viewCenterX = centerX;
        viewCenterY = centerY;
        viewCenterInitialized = true;
        repaint();
    }

    void mouseUp(const juce::MouseEvent&) override
    {
        const bool hadSurfacePoint = draggedSurfaceLensIndex >= 0 && draggedSurfaceFocusIndex >= 0;
        const bool hadSurfaceDragSession = surfaceDragSessionActive;
        const auto releasedSurfaceLensIndex = draggedSurfaceLensIndex;
        const auto releasedSurfaceFocusIndex = draggedSurfaceFocusIndex;
        draggedSurfaceLensIndex = -1;
        draggedSurfaceFocusIndex = -1;
        surfaceDragSessionActive = false;
        if (hadSurfaceDragSession && onSurfaceDragEnded)
            onSurfaceDragEnded();

        if (hadSurfacePoint
            && !dragMovedSinceMouseDown
            && releasedSurfaceLensIndex == mouseDownSurfaceLensIndex
            && releasedSurfaceFocusIndex == mouseDownSurfaceFocusIndex
            && onSurfacePointClicked)
        {
            onSurfacePointClicked(
                static_cast<size_t>(releasedSurfaceLensIndex),
                static_cast<size_t>(releasedSurfaceFocusIndex));
        }

        mouseDownSurfaceLensIndex = -1;
        mouseDownSurfaceFocusIndex = -1;

        const bool hadPoint = draggedPointIndex >= 0;
        const bool hadDragSession = dragSessionActive;
        const auto releasedPointIndex = draggedPointIndex;
        draggedPointIndex = -1;
        dragSessionActive = false;
        if (hadDragSession && onLutDragEnded)
            onLutDragEnded();

        if (hadPoint && !dragMovedSinceMouseDown && releasedPointIndex == mouseDownPointIndex && onLutPointClicked)
            onLutPointClicked(static_cast<size_t>(releasedPointIndex));

        mouseDownPointIndex = -1;
        panSessionActive = false;
        dragMovedSinceMouseDown = false;
    }

    void paint(juce::Graphics& g) override
    {
        g.fillAll(juce::Colours::black);

        auto area = getLocalBounds().reduced(6).toFloat();
        g.setColour(juce::Colours::darkgrey);
        g.drawRect(area);

        auto plot = area.reduced(8.0f, 8.0f);

        float minX = 0.0f;
        float maxX = 1.0f;
        float minY = 0.0f;
        float maxY = 1.0f;
        getViewRanges(minX, maxX, minY, maxY);

        const bool hasSurface = hasCompleteSurfaceData();

        if (hasSurface)
        {
            const auto lensCount = static_cast<int>(surfaceLensAxis.size());
            const auto focusCount = static_cast<int>(surfaceFocusAxis.size());
            for (int j = 0; j < focusCount; ++j)
            {
                juce::Path surfacePath;
                const auto colour = juce::Colour::fromHSV(
                    static_cast<float>(j) / juce::jmax(1, focusCount),
                    0.55f,
                    0.95f,
                    0.7f);
                for (int i = 0; i < lensCount; ++i)
                {
                    const auto x = juce::jmap(surfaceLensAxis[static_cast<size_t>(i)], minX, maxX, plot.getX(), plot.getRight());
                    const auto y = juce::jmap(surfaceValuesGrid[static_cast<size_t>(i * focusCount + j)], maxY, minY, plot.getY(), plot.getBottom());
                    if (i == 0)
                        surfacePath.startNewSubPath(x, y);
                    else
                        surfacePath.lineTo(x, y);
                }

                g.setColour(colour);
                g.strokePath(surfacePath, juce::PathStrokeType(1.2f));
            }

            for (int i = 0; i < lensCount; ++i)
            {
                for (int j = 0; j < focusCount; ++j)
                {
                    const auto pointColour = juce::Colour::fromHSV(
                        static_cast<float>(j) / juce::jmax(1, focusCount),
                        0.75f,
                        1.0f,
                        0.95f);
                    const auto x = juce::jmap(surfaceLensAxis[static_cast<size_t>(i)], minX, maxX, plot.getX(), plot.getRight());
                    const auto y = juce::jmap(surfaceValuesGrid[static_cast<size_t>(i * focusCount + j)], maxY, minY, plot.getY(), plot.getBottom());
                    g.setColour(juce::Colours::black.withAlpha(0.9f));
                    g.fillEllipse(x - 5.2f, y - 5.2f, 10.4f, 10.4f);
                    g.setColour(juce::Colours::white.withAlpha(0.98f));
                    g.fillEllipse(x - 4.1f, y - 4.1f, 8.2f, 8.2f);
                    g.setColour(pointColour);
                    g.fillEllipse(x - 2.8f, y - 2.8f, 5.6f, 5.6f);
                }
            }

            if (selectedSurfaceLensIndex >= 0
                && selectedSurfaceFocusIndex >= 0
                && selectedSurfaceLensIndex < lensCount
                && selectedSurfaceFocusIndex < focusCount)
            {
                const auto x = juce::jmap(
                    surfaceLensAxis[static_cast<size_t>(selectedSurfaceLensIndex)],
                    minX, maxX, plot.getX(), plot.getRight());
                const auto y = juce::jmap(
                    surfaceValuesGrid[static_cast<size_t>(selectedSurfaceLensIndex * focusCount + selectedSurfaceFocusIndex)],
                    maxY, minY, plot.getY(), plot.getBottom());
                g.setColour(juce::Colours::deepskyblue.withAlpha(0.95f));
                g.drawEllipse(x - 6.5f, y - 6.5f, 13.0f, 13.0f, 2.0f);
            }
        }
        else
        {
            juce::Path lutPath;
            const int pointCount = lutOutputValues.empty() ? 2 : static_cast<int>(lutOutputValues.size());
            const bool hasExplicitX = (!lutInputValues.empty() && lutInputValues.size() == lutOutputValues.size());
            for (int i = 0; i < pointCount; ++i)
            {
                const auto t = pointCount == 1 ? 0.0f : static_cast<float>(i) / static_cast<float>(pointCount - 1);
                const auto xValue = hasExplicitX
                    ? lutInputValues[static_cast<size_t>(i)]
                    : (minX + t * (maxX - minX));
                const auto yValue = lutOutputValues.empty() ? xValue : lutOutputValues[static_cast<size_t>(i)];

                const auto x = juce::jmap(xValue, minX, maxX, plot.getX(), plot.getRight());
                const auto y = juce::jmap(yValue, maxY, minY, plot.getY(), plot.getBottom());

                if (i == 0)
                    lutPath.startNewSubPath(x, y);
                else
                    lutPath.lineTo(x, y);
            }

            g.setColour(juce::Colours::lightgreen);
            g.strokePath(lutPath, juce::PathStrokeType(2.0f));

            // Show the LUT table samples explicitly.
            g.setColour(juce::Colours::white.withAlpha(0.9f));
            for (int i = 0; i < pointCount; ++i)
            {
                const auto t = pointCount == 1 ? 0.0f : static_cast<float>(i) / static_cast<float>(pointCount - 1);
                const auto xValue = hasExplicitX
                    ? lutInputValues[static_cast<size_t>(i)]
                    : (minX + t * (maxX - minX));
                const auto yValue = lutOutputValues.empty() ? xValue : lutOutputValues[static_cast<size_t>(i)];
                const auto x = juce::jmap(xValue, minX, maxX, plot.getX(), plot.getRight());
                const auto y = juce::jmap(yValue, maxY, minY, plot.getY(), plot.getBottom());
                g.fillEllipse(x - 1.8f, y - 1.8f, 3.6f, 3.6f);
            }

            if (selectedPointIndex >= 0 && selectedPointIndex < pointCount)
            {
                const auto i = selectedPointIndex;
                const auto t = pointCount == 1 ? 0.0f : static_cast<float>(i) / static_cast<float>(pointCount - 1);
                const auto xValue = hasExplicitX
                    ? lutInputValues[static_cast<size_t>(i)]
                    : (minX + t * (maxX - minX));
                const auto yValue = lutOutputValues.empty() ? xValue : lutOutputValues[static_cast<size_t>(i)];
                const auto x = juce::jmap(xValue, minX, maxX, plot.getX(), plot.getRight());
                const auto y = juce::jmap(yValue, maxY, minY, plot.getY(), plot.getBottom());

                g.setColour(juce::Colours::deepskyblue.withAlpha(0.95f));
                g.drawEllipse(x - 6.0f, y - 6.0f, 12.0f, 12.0f, 1.8f);
            }
        }

        if (!cubicCurveX.empty() && cubicCurveX.size() == cubicCurveY.size())
        {
            juce::Path cubicPath;
            for (size_t i = 0; i < cubicCurveX.size(); ++i)
            {
                const auto x = juce::jmap(cubicCurveX[i], minX, maxX, plot.getX(), plot.getRight());
                const auto y = juce::jmap(cubicCurveY[i], maxY, minY, plot.getY(), plot.getBottom());
                if (i == 0)
                    cubicPath.startNewSubPath(x, y);
                else
                    cubicPath.lineTo(x, y);
            }

            g.setColour(juce::Colours::yellow.withAlpha(0.95f));
            g.strokePath(cubicPath, juce::PathStrokeType(1.5f));
        }

        const auto inputClampedX = juce::jlimit(minX, maxX, inputValue);
        const auto xInput = juce::jmap(inputClampedX, minX, maxX, plot.getX(), plot.getRight());
        const auto lutSampleClampedY = juce::jlimit(minY, maxY, lutSampleValue);
        const auto cubicOutputClampedY = juce::jlimit(minY, maxY, cubicOutputValue);
        const auto yInput = plot.getBottom();
        const auto yLutSample = juce::jmap(lutSampleClampedY, maxY, minY, plot.getY(), plot.getBottom());
        const auto yCubicOutput = juce::jmap(cubicOutputClampedY, maxY, minY, plot.getY(), plot.getBottom());

        g.setColour(juce::Colours::white.withAlpha(0.25f));
        g.drawLine(xInput, plot.getY(), xInput, plot.getBottom(), 1.0f);

        g.setColour(juce::Colours::deepskyblue);
        g.fillEllipse(xInput - 4.0f, yInput - 4.0f, 8.0f, 8.0f); // input point

        g.setColour(juce::Colours::lightgreen);
        g.fillEllipse(xInput - 4.0f, yLutSample - 4.0f, 8.0f, 8.0f); // linear LUT output

        g.setColour(juce::Colours::orange);
        g.fillEllipse(xInput - 5.0f, yCubicOutput - 5.0f, 10.0f, 10.0f); // cubic interpolated output

        auto legend = area.removeFromTop(16).toNearestInt();
        g.setColour(juce::Colours::deepskyblue);
        g.drawText("Input", legend.removeFromLeft(90), juce::Justification::left);
        g.setColour(juce::Colours::lightgreen);
        g.drawText(hasSurface ? "Surface" : "LUT", legend.removeFromLeft(80), juce::Justification::left);
        g.setColour(juce::Colours::yellow.withAlpha(0.95f));
        g.drawText("Cubic curve", legend.removeFromLeft(110), juce::Justification::left);
        g.setColour(juce::Colours::lightgreen);
        g.drawText("LUT linear", legend.removeFromLeft(100), juce::Justification::left);
        g.setColour(juce::Colours::orange);
        g.drawText("Cubic out", legend.removeFromLeft(90), juce::Justification::left);
        g.setColour(juce::Colours::white.withAlpha(0.8f));
        g.drawText("Input: " + juce::String(inputValue, 4), legend, juce::Justification::right);
    }

private:
    void getViewRanges(float& minXOut, float& maxXOut, float& minYOut, float& maxYOut)
    {
        auto fullMinX = xMin;
        auto fullMaxX = xMax;
        if (fullMaxX <= fullMinX)
        {
            fullMinX = 0.0f;
            fullMaxX = 1.0f;
        }

        auto fullMinY = yMin;
        auto fullMaxY = yMax;
        if (fullMaxY <= fullMinY)
        {
            fullMinY -= 1.0f;
            fullMaxY += 1.0f;
        }

        if (!viewCenterInitialized
            || !std::isfinite(viewCenterX)
            || !std::isfinite(viewCenterY))
        {
            viewCenterX = 0.5f * (fullMinX + fullMaxX);
            viewCenterY = 0.5f * (fullMinY + fullMaxY);
            viewCenterInitialized = true;
        }

        const auto z = juce::jmax(1.0f, zoomFactor);
        const auto halfX = 0.5f * (fullMaxX - fullMinX) / z;
        const auto halfY = 0.5f * (fullMaxY - fullMinY) / z;
        const auto minCenterX = fullMinX + halfX;
        const auto maxCenterX = fullMaxX - halfX;
        const auto minCenterY = fullMinY + halfY;
        const auto maxCenterY = fullMaxY - halfY;
        if (maxCenterX > minCenterX)
            viewCenterX = juce::jlimit(minCenterX, maxCenterX, viewCenterX);
        else
            viewCenterX = 0.5f * (fullMinX + fullMaxX);
        if (maxCenterY > minCenterY)
            viewCenterY = juce::jlimit(minCenterY, maxCenterY, viewCenterY);
        else
            viewCenterY = 0.5f * (fullMinY + fullMaxY);

        auto minX = fullMinX;
        auto maxX = fullMaxX;
        auto minY = fullMinY;
        auto maxY = fullMaxY;
        if (z > 1.0001f)
        {
            minX = viewCenterX - halfX;
            maxX = viewCenterX + halfX;
            minY = viewCenterY - halfY;
            maxY = viewCenterY + halfY;
        }

        minXOut = minX;
        maxXOut = maxX;
        minYOut = minY;
        maxYOut = maxY;
    }

    void updateStaticYRange()
    {
        if (hasCompleteSurfaceData())
        {
            const auto [minIt, maxIt] = std::minmax_element(surfaceValuesGrid.begin(), surfaceValuesGrid.end());
            yMin = *minIt;
            yMax = *maxIt;
        }
        else if (!lutOutputValues.empty())
        {
            const auto [minIt, maxIt] = std::minmax_element(lutOutputValues.begin(), lutOutputValues.end());
            yMin = *minIt;
            yMax = *maxIt;
        }
        else
        {
            yMin = xMin;
            yMax = xMax;
        }

        if (yMax <= yMin)
        {
            yMin -= 1.0f;
            yMax += 1.0f;
        }
    }

    bool hasCompleteSurfaceData() const
    {
        return !surfaceLensAxis.empty()
            && !surfaceFocusAxis.empty()
            && surfaceValuesGrid.size() == surfaceLensAxis.size() * surfaceFocusAxis.size();
    }

    std::vector<float> lutInputValues;
    std::vector<float> lutOutputValues;
    std::vector<float> surfaceLensAxis;
    std::vector<float> surfaceFocusAxis;
    std::vector<float> surfaceValuesGrid;
    std::vector<float> cubicCurveX;
    std::vector<float> cubicCurveY;
    float xMin = 0.0f;
    float xMax = 1.0f;
    float yMin = 0.0f;
    float yMax = 1.0f;
    float inputValue = 0.0f;
    float lutSampleValue = 0.0f;
    float cubicOutputValue = 0.0f;
    int selectedPointIndex = -1;
    int selectedSurfaceLensIndex = -1;
    int selectedSurfaceFocusIndex = -1;
    int draggedPointIndex = -1;
    int mouseDownPointIndex = -1;
    int draggedSurfaceLensIndex = -1;
    int draggedSurfaceFocusIndex = -1;
    int mouseDownSurfaceLensIndex = -1;
    int mouseDownSurfaceFocusIndex = -1;
    bool dragMovedSinceMouseDown = false;
    bool dragSessionActive = false;
    bool surfaceDragSessionActive = false;
    float zoomFactor = 1.0f;
    bool panSessionActive = false;
    bool viewCenterInitialized = false;
    float viewCenterX = 0.0f;
    float viewCenterY = 0.0f;
    float panStartZoomFactor = 1.0f;
    float panStartCenterX = 0.0f;
    float panStartCenterY = 0.0f;
    float panStartViewWidth = 1.0f;
    float panStartViewHeight = 1.0f;
    juce::Point<float> panStartMousePosition;
    juce::Point<float> mouseDownPosition;
};

class MainComponent final : public juce::Component,
                            private juce::Timer,
                            private juce::OSCReceiver,
                            private juce::OSCReceiver::Listener<juce::OSCReceiver::MessageLoopCallback>
{
public:
    struct LutSnapshot
    {
        std::vector<float> inputValues;
        std::vector<float> outputValues;
    };

    MainComponent()
        : applyButton("Apply Connections"),
          loadLutButton("Load LUT"),
          saveLutButton("Save LUT"),
          loadSurfaceButton("Load Surface"),
          newLutButton("New LUT"),
          addPointAtInputButton("Add Pt @ In"),
          undoLutButton("Undo LUT"),
          redoLutButton("Redo LUT"),
          loadPresetButton("Load Preset"),
          savePresetButton("Save Preset")
    {
        titleLabel.setText("OSC In -> Interpolation -> OSC Out", juce::dontSendNotification);
        titleLabel.setJustificationType(juce::Justification::centred);
        addAndMakeVisible(titleLabel);

        inPortLabel.setText("In Port", juce::dontSendNotification);
        outHostLabel.setText("Out Host", juce::dontSendNotification);
        outPortLabel.setText("Out Port", juce::dontSendNotification);
        freeDPortLabel.setText("FreeD Port", juce::dontSendNotification);
        inAddressLabel.setText("In Address", juce::dontSendNotification);
        focusAddressLabel.setText("Focus Addr", juce::dontSendNotification);
        focusInMinLabel.setText("Focus In Min", juce::dontSendNotification);
        focusInMaxLabel.setText("Focus In Max", juce::dontSendNotification);
        outAddressLabel.setText("Out Address", juce::dontSendNotification);

        addAndMakeVisible(inPortLabel);
        addAndMakeVisible(outHostLabel);
        addAndMakeVisible(outPortLabel);
        addAndMakeVisible(freeDPortLabel);
        addAndMakeVisible(inAddressLabel);
        addAndMakeVisible(focusAddressLabel);
        addAndMakeVisible(focusInMinLabel);
        addAndMakeVisible(focusInMaxLabel);
        addAndMakeVisible(outAddressLabel);

        inPortEditor.setText(juce::String(kDefaultInputPort), juce::dontSendNotification);
        outHostEditor.setText("127.0.0.1", juce::dontSendNotification);
        outPortEditor.setText(juce::String(kDefaultOutputPort), juce::dontSendNotification);
        freeDPortEditor.setText(juce::String(kDefaultFreeDPort), juce::dontSendNotification);
        inAddressEditor.setText("/input", juce::dontSendNotification);
        focusAddressEditor.setText("/focus", juce::dontSendNotification);
        focusInMinEditor.setText("0.0", juce::dontSendNotification);
        focusInMaxEditor.setText("1.0", juce::dontSendNotification);
        outAddressEditor.setText("/output", juce::dontSendNotification);
        freeDEnableToggle.setButtonText("Use FreeD UDP");
        freeDEnableToggle.setToggleState(false, juce::dontSendNotification);
        restartFreeDButton.setButtonText("Restart FreeD");

        addAndMakeVisible(inPortEditor);
        addAndMakeVisible(outHostEditor);
        addAndMakeVisible(outPortEditor);
        addAndMakeVisible(freeDPortEditor);
        addAndMakeVisible(inAddressEditor);
        addAndMakeVisible(focusAddressEditor);
        addAndMakeVisible(focusInMinEditor);
        addAndMakeVisible(focusInMaxEditor);
        addAndMakeVisible(outAddressEditor);
        addAndMakeVisible(freeDEnableToggle);
        addAndMakeVisible(restartFreeDButton);

        inputMinLabel.setText("Input Min", juce::dontSendNotification);
        inputMaxLabel.setText("Input Max", juce::dontSendNotification);
        inputMinEditor.setText("0.0", juce::dontSendNotification);
        inputMaxEditor.setText("1.0", juce::dontSendNotification);
        addAndMakeVisible(inputMinLabel);
        addAndMakeVisible(inputMaxLabel);
        addAndMakeVisible(inputMinEditor);
        addAndMakeVisible(inputMaxEditor);

        addAndMakeVisible(loadLutButton);
        addAndMakeVisible(saveLutButton);
        addAndMakeVisible(loadSurfaceButton);
        addAndMakeVisible(newLutButton);
        addAndMakeVisible(addPointAtInputButton);
        addAndMakeVisible(undoLutButton);
        addAndMakeVisible(redoLutButton);
        lutStatusLabel.setText("LUT: passthrough (load 4+ values or x,y pairs)", juce::dontSendNotification);
        addAndMakeVisible(lutStatusLabel);

        pointEditLabel.setText("Point Edit", juce::dontSendNotification);
        pointEditValueEditor.setEnabled(false);
        pointEditValueEditor.setText("-", juce::dontSendNotification);
        pointEditValueEditor.setInputRestrictions(0, "0123456789+-.eE");
        pointEditValueEditor.onTextChange = [this] { handlePointEditorTextChange(); };
        pointDec1Button.setButtonText("-1.0");
        pointInc1Button.setButtonText("+1.0");
        pointDec01Button.setButtonText("-0.1");
        pointInc01Button.setButtonText("+0.1");
        pointDec001Button.setButtonText("-0.01");
        pointInc001Button.setButtonText("+0.01");
        pointDec1Button.onClick = [this] { nudgeSelectedPoint(-1.0f); };
        pointInc1Button.onClick = [this] { nudgeSelectedPoint(+1.0f); };
        pointDec01Button.onClick = [this] { nudgeSelectedPoint(-0.1f); };
        pointInc01Button.onClick = [this] { nudgeSelectedPoint(+0.1f); };
        pointDec001Button.onClick = [this] { nudgeSelectedPoint(-0.01f); };
        pointInc001Button.onClick = [this] { nudgeSelectedPoint(+0.01f); };
        addAndMakeVisible(pointEditLabel);
        addAndMakeVisible(pointEditValueEditor);
        addAndMakeVisible(pointDec1Button);
        addAndMakeVisible(pointInc1Button);
        addAndMakeVisible(pointDec01Button);
        addAndMakeVisible(pointInc01Button);
        addAndMakeVisible(pointDec001Button);
        addAndMakeVisible(pointInc001Button);

        graphZoomLabel.setText("Graph Zoom", juce::dontSendNotification);
        graphZoomSlider.setSliderStyle(juce::Slider::LinearHorizontal);
        graphZoomSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 72, 24);
        graphZoomSlider.setRange(1.0, 40.0, 0.01);
        graphZoomSlider.setSkewFactorFromMidPoint(4.0);
        graphZoomSlider.setValue(1.0);
        graphZoomSlider.onValueChange = [this]
        {
            lutCartesianPlot.setZoomFactor(static_cast<float>(graphZoomSlider.getValue()));
        };
        resetGraphZoomButton.setButtonText("Reset Zoom");
        resetGraphZoomButton.onClick = [this]
        {
            graphZoomSlider.setValue(1.0, juce::sendNotificationSync);
        };
        addAndMakeVisible(graphZoomLabel);
        addAndMakeVisible(graphZoomSlider);
        addAndMakeVisible(resetGraphZoomButton);

        smoothingLabel.setText("Smoothing (ms)", juce::dontSendNotification);
        outputRateLabel.setText("Output Rate (Hz)", juce::dontSendNotification);
        addAndMakeVisible(smoothingLabel);
        addAndMakeVisible(outputRateLabel);

        smoothingSlider.setSliderStyle(juce::Slider::LinearHorizontal);
        smoothingSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 72, 24);
        smoothingSlider.setRange(0.0, 5000.0, 1.0);
        smoothingSlider.setValue(kDefaultSmoothingMs);
        addAndMakeVisible(smoothingSlider);

        outputRateSlider.setSliderStyle(juce::Slider::LinearHorizontal);
        outputRateSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 72, 24);
        outputRateSlider.setRange(1.0, 240.0, 1.0);
        outputRateSlider.setValue(kDefaultOutputRateHz);
        addAndMakeVisible(outputRateSlider);

        presetNameLabel.setText("Preset", juce::dontSendNotification);
        presetNameEditor.setText("default", juce::dontSendNotification);
        settingsStatusLabel.setText("Settings: not loaded", juce::dontSendNotification);
        addAndMakeVisible(presetNameLabel);
        addAndMakeVisible(presetNameEditor);
        addAndMakeVisible(loadPresetButton);
        addAndMakeVisible(savePresetButton);
        addAndMakeVisible(settingsStatusLabel);

        rawValueLabel.setText("Raw: 0.00000", juce::dontSendNotification);
        focusValueLabel.setText("Focus: 0.00000", juce::dontSendNotification);
        freeDStatusLabel.setText("FreeD: disabled", juce::dontSendNotification);
        targetValueLabel.setText("Target: 0.00000", juce::dontSendNotification);
        interpValueLabel.setText("Interpolated: 0.00000", juce::dontSendNotification);
        receivedCountLabel.setText("Received Messages: 0", juce::dontSendNotification);
        connectionStatusLabel.setText("Input: disconnected | Output: disconnected", juce::dontSendNotification);

        addAndMakeVisible(rawValueLabel);
        addAndMakeVisible(focusValueLabel);
        addAndMakeVisible(freeDStatusLabel);
        addAndMakeVisible(targetValueLabel);
        addAndMakeVisible(interpValueLabel);
        addAndMakeVisible(receivedCountLabel);
        addAndMakeVisible(connectionStatusLabel);
        addAndMakeVisible(historyPlot);
        addAndMakeVisible(lutCartesianPlot);

        lutCartesianPlot.onLutPointDragged = [this](size_t index, float outputValue)
        {
            handleLutPointDragged(index, outputValue);
        };
        lutCartesianPlot.onLutDragStarted = [this]
        {
            beginLutDragHistory();
        };
        lutCartesianPlot.onLutDragEnded = [this]
        {
            endLutDragHistory();
        };
        lutCartesianPlot.onLutPointClicked = [this](size_t index)
        {
            selectLutPointForEditing(index);
        };
        lutCartesianPlot.onSurfacePointDragged = [this](size_t lensIndex, size_t focusIndex, float outputValue)
        {
            handleSurfacePointDragged(lensIndex, focusIndex, outputValue);
        };
        lutCartesianPlot.onSurfacePointClicked = [this](size_t lensIndex, size_t focusIndex)
        {
            selectSurfacePointForEditing(lensIndex, focusIndex);
        };
        lutCartesianPlot.onZoomChanged = [this](float zoom)
        {
            graphZoomSlider.setValue(zoom, juce::dontSendNotification);
        };

        applyButton.onClick = [this] { applyConnections(); };
        addAndMakeVisible(applyButton);
        restartFreeDButton.onClick = [this] { restartFreeDListener(); };

        loadLutButton.onClick = [this] { loadLutFromFile(); };
        saveLutButton.onClick = [this] { saveLutToFile(); };
        loadSurfaceButton.onClick = [this] { loadSurfaceFromFile(); };
        newLutButton.onClick = [this] { startNewLutCreation(); };
        addPointAtInputButton.onClick = [this] { addLutPointAtCurrentInput(); };
        undoLutButton.onClick = [this] { undoLutChange(); };
        redoLutButton.onClick = [this] { redoLutChange(); };
        loadPresetButton.onClick = [this] { loadCurrentPreset(); };
        savePresetButton.onClick = [this] { saveCurrentPreset(false); };
        outputRateSlider.onValueChange = [this] { restartOutputTimer(); };

        addListener(this);
        loadLastPresetFromSettings();
        applyConnections();
        restartOutputTimer();
        updateUndoRedoButtons();

        setSize(760, 1020);
    }

    ~MainComponent() override
    {
        stopTimer();
        saveCurrentPreset(true);
        sender.disconnect();
        closeFreeDConnection();
        disconnect();
        removeListener(this);
    }

    void resized() override
    {
        auto area = getLocalBounds().reduced(12);
        titleLabel.setBounds(area.removeFromTop(28));
        area.removeFromTop(8);

        const int rowH = 28;
        const int labelW = 90;
        const int shortEditorW = 90;
        const int mediumEditorW = 170;
        const int buttonW = 170;
        const int lutButtonW = 110;
        const int newLutButtonW = 100;
        const int addPointButtonW = 120;
        const int pointButtonW = 62;
        const int undoRedoButtonW = 110;
        const int presetButtonW = 130;
        const int presetEditorW = 140;
        const int gap = 8;

        auto row1 = area.removeFromTop(rowH);
        inPortLabel.setBounds(row1.removeFromLeft(labelW));
        inPortEditor.setBounds(row1.removeFromLeft(shortEditorW));
        row1.removeFromLeft(gap);
        outHostLabel.setBounds(row1.removeFromLeft(labelW));
        outHostEditor.setBounds(row1.removeFromLeft(mediumEditorW));
        row1.removeFromLeft(gap);
        outPortLabel.setBounds(row1.removeFromLeft(labelW));
        outPortEditor.setBounds(row1.removeFromLeft(shortEditorW));

        area.removeFromTop(6);

        auto row1b = area.removeFromTop(rowH);
        freeDEnableToggle.setBounds(row1b.removeFromLeft(140));
        row1b.removeFromLeft(gap);
        freeDPortLabel.setBounds(row1b.removeFromLeft(labelW));
        freeDPortEditor.setBounds(row1b.removeFromLeft(shortEditorW));
        row1b.removeFromLeft(gap);
        restartFreeDButton.setBounds(row1b.removeFromLeft(130));

        area.removeFromTop(6);

        auto row2 = area.removeFromTop(rowH);
        inAddressLabel.setBounds(row2.removeFromLeft(labelW));
        inAddressEditor.setBounds(row2.removeFromLeft(180));
        row2.removeFromLeft(gap);
        outAddressLabel.setBounds(row2.removeFromLeft(labelW));
        outAddressEditor.setBounds(row2.removeFromLeft(180));

        area.removeFromTop(6);

        auto row2b = area.removeFromTop(rowH);
        focusAddressLabel.setBounds(row2b.removeFromLeft(labelW));
        focusAddressEditor.setBounds(row2b.removeFromLeft(180));
        row2b.removeFromLeft(gap);
        applyButton.setBounds(row2b.removeFromLeft(buttonW));

        area.removeFromTop(6);

        auto row2c = area.removeFromTop(rowH);
        focusInMinLabel.setBounds(row2c.removeFromLeft(labelW));
        focusInMinEditor.setBounds(row2c.removeFromLeft(shortEditorW));
        row2c.removeFromLeft(gap);
        focusInMaxLabel.setBounds(row2c.removeFromLeft(labelW));
        focusInMaxEditor.setBounds(row2c.removeFromLeft(shortEditorW));

        area.removeFromTop(6);

        auto row3 = area.removeFromTop(rowH);
        inputMinLabel.setBounds(row3.removeFromLeft(labelW));
        inputMinEditor.setBounds(row3.removeFromLeft(shortEditorW));
        row3.removeFromLeft(gap);
        inputMaxLabel.setBounds(row3.removeFromLeft(labelW));
        inputMaxEditor.setBounds(row3.removeFromLeft(shortEditorW));
        row3.removeFromLeft(gap);
        loadLutButton.setBounds(row3.removeFromLeft(lutButtonW));
        row3.removeFromLeft(gap);
        saveLutButton.setBounds(row3.removeFromLeft(lutButtonW));
        row3.removeFromLeft(gap);
        loadSurfaceButton.setBounds(row3.removeFromLeft(lutButtonW));

        area.removeFromTop(6);
        auto row3b = area.removeFromTop(rowH);
        newLutButton.setBounds(row3b.removeFromLeft(newLutButtonW));
        row3b.removeFromLeft(gap);
        addPointAtInputButton.setBounds(row3b.removeFromLeft(addPointButtonW));
        row3b.removeFromLeft(gap);
        undoLutButton.setBounds(row3b.removeFromLeft(undoRedoButtonW));
        row3b.removeFromLeft(gap);
        redoLutButton.setBounds(row3b.removeFromLeft(undoRedoButtonW));

        area.removeFromTop(6);
        lutStatusLabel.setBounds(area.removeFromTop(24));

        area.removeFromTop(6);
        auto row3c = area.removeFromTop(rowH);
        pointEditLabel.setBounds(row3c.removeFromLeft(labelW));
        pointEditValueEditor.setBounds(row3c.removeFromLeft(90));
        row3c.removeFromLeft(gap);
        pointDec1Button.setBounds(row3c.removeFromLeft(pointButtonW));
        row3c.removeFromLeft(4);
        pointInc1Button.setBounds(row3c.removeFromLeft(pointButtonW));
        row3c.removeFromLeft(6);
        pointDec01Button.setBounds(row3c.removeFromLeft(pointButtonW));
        row3c.removeFromLeft(4);
        pointInc01Button.setBounds(row3c.removeFromLeft(pointButtonW));
        row3c.removeFromLeft(6);
        pointDec001Button.setBounds(row3c.removeFromLeft(pointButtonW));
        row3c.removeFromLeft(4);
        pointInc001Button.setBounds(row3c.removeFromLeft(pointButtonW));

        area.removeFromTop(6);
        auto row3d = area.removeFromTop(rowH);
        graphZoomLabel.setBounds(row3d.removeFromLeft(labelW));
        graphZoomSlider.setBounds(row3d.removeFromLeft(280));
        row3d.removeFromLeft(gap);
        resetGraphZoomButton.setBounds(row3d.removeFromLeft(110));

        area.removeFromTop(10);

        auto row4 = area.removeFromTop(rowH);
        smoothingLabel.setBounds(row4.removeFromLeft(120));
        smoothingSlider.setBounds(row4);

        area.removeFromTop(6);

        auto row5 = area.removeFromTop(rowH);
        outputRateLabel.setBounds(row5.removeFromLeft(120));
        outputRateSlider.setBounds(row5);

        area.removeFromTop(8);

        auto row6 = area.removeFromTop(rowH);
        presetNameLabel.setBounds(row6.removeFromLeft(labelW));
        presetNameEditor.setBounds(row6.removeFromLeft(presetEditorW));
        row6.removeFromLeft(gap);
        loadPresetButton.setBounds(row6.removeFromLeft(presetButtonW));
        row6.removeFromLeft(gap);
        savePresetButton.setBounds(row6.removeFromLeft(presetButtonW));

        area.removeFromTop(6);
        settingsStatusLabel.setBounds(area.removeFromTop(24));

        area.removeFromTop(10);

        auto historyArea = area.removeFromTop(170);
        historyPlot.setBounds(historyArea);
        area.removeFromTop(8);

        auto lutArea = area.removeFromTop(170);
        lutCartesianPlot.setBounds(lutArea);
        area.removeFromTop(8);

        rawValueLabel.setBounds(area.removeFromTop(24));
        focusValueLabel.setBounds(area.removeFromTop(24));
        freeDStatusLabel.setBounds(area.removeFromTop(24));
        targetValueLabel.setBounds(area.removeFromTop(24));
        interpValueLabel.setBounds(area.removeFromTop(24));
        receivedCountLabel.setBounds(area.removeFromTop(24));
        area.removeFromTop(4);
        connectionStatusLabel.setBounds(area.removeFromTop(24));
    }

private:
    juce::File getSettingsFile() const
    {
        return juce::File::getSpecialLocation(juce::File::userApplicationDataDirectory)
            .getChildFile("LinInterpol")
            .getChildFile("lininterpol_presets.xml");
    }

    std::unique_ptr<juce::XmlElement> loadSettingsXml() const
    {
        const auto settingsFile = getSettingsFile();
        if (!settingsFile.existsAsFile())
            return {};

        return juce::parseXML(settingsFile);
    }

    static juce::XmlElement* findPresetByName(juce::XmlElement& root, const juce::String& presetName)
    {
        for (auto* child = root.getFirstChildElement(); child != nullptr; child = child->getNextElement())
        {
            if (child->hasTagName(kPresetTag) && child->getStringAttribute("name") == presetName)
                return child;
        }
        return nullptr;
    }

    bool writeSettingsXml(const juce::XmlElement& root, juce::String& error) const
    {
        const auto settingsFile = getSettingsFile();
        const auto parentDir = settingsFile.getParentDirectory();
        if (!parentDir.exists() && !parentDir.createDirectory())
        {
            error = "Could not create settings directory: " + parentDir.getFullPathName();
            return false;
        }

        if (!settingsFile.replaceWithText(root.toString()))
        {
            error = "Could not write settings file: " + settingsFile.getFullPathName();
            return false;
        }
        return true;
    }

    juce::String currentPresetName() const
    {
        const auto name = presetNameEditor.getText().trim();
        return name.isEmpty() ? juce::String("default") : name;
    }

    bool savePreset(const juce::String& presetName, juce::String& error)
    {
        auto settingsXml = loadSettingsXml();
        if (settingsXml == nullptr || !settingsXml->hasTagName(kSettingsRootTag))
            settingsXml = std::make_unique<juce::XmlElement>(kSettingsRootTag);

        auto* preset = findPresetByName(*settingsXml, presetName);
        if (preset == nullptr)
        {
            preset = settingsXml->createNewChildElement(kPresetTag);
            preset->setAttribute("name", presetName);
        }

        preset->setAttribute("inPort", inPortEditor.getText().trim());
        preset->setAttribute("outHost", outHostEditor.getText().trim());
        preset->setAttribute("outPort", outPortEditor.getText().trim());
        preset->setAttribute("freeDEnabled", freeDEnableToggle.getToggleState() ? 1 : 0);
        preset->setAttribute("freeDPort", freeDPortEditor.getText().trim());
        preset->setAttribute("inAddress", inAddressEditor.getText().trim());
        preset->setAttribute("focusAddress", focusAddressEditor.getText().trim());
        preset->setAttribute("focusInMin", focusInMinEditor.getText().trim());
        preset->setAttribute("focusInMax", focusInMaxEditor.getText().trim());
        preset->setAttribute("outAddress", outAddressEditor.getText().trim());
        preset->setAttribute("inputMin", inputMinEditor.getText().trim());
        preset->setAttribute("inputMax", inputMaxEditor.getText().trim());
        preset->setAttribute("smoothingMs", smoothingSlider.getValue());
        preset->setAttribute("outputRateHz", outputRateSlider.getValue());
        if (lastLutFilePath.isNotEmpty())
            preset->setAttribute("lutFile", lastLutFilePath);
        else
            preset->removeAttribute("lutFile");
        if (lastSurfaceFilePath.isNotEmpty())
            preset->setAttribute("surfaceFile", lastSurfaceFilePath);
        else
            preset->removeAttribute("surfaceFile");

        settingsXml->setAttribute("lastPreset", presetName);
        return writeSettingsXml(*settingsXml, error);
    }

    bool loadPreset(const juce::String& presetName, juce::String& error)
    {
        auto settingsXml = loadSettingsXml();
        if (settingsXml == nullptr || !settingsXml->hasTagName(kSettingsRootTag))
        {
            error = "Settings file not found";
            return false;
        }

        auto* preset = findPresetByName(*settingsXml, presetName);
        if (preset == nullptr)
        {
            error = "Preset not found: " + presetName;
            return false;
        }

        inPortEditor.setText(preset->getStringAttribute("inPort", inPortEditor.getText()), juce::dontSendNotification);
        outHostEditor.setText(preset->getStringAttribute("outHost", outHostEditor.getText()), juce::dontSendNotification);
        outPortEditor.setText(preset->getStringAttribute("outPort", outPortEditor.getText()), juce::dontSendNotification);
        freeDEnableToggle.setToggleState(preset->getIntAttribute("freeDEnabled", freeDEnableToggle.getToggleState() ? 1 : 0) != 0, juce::dontSendNotification);
        freeDPortEditor.setText(preset->getStringAttribute("freeDPort", freeDPortEditor.getText()), juce::dontSendNotification);
        inAddressEditor.setText(preset->getStringAttribute("inAddress", inAddressEditor.getText()), juce::dontSendNotification);
        focusAddressEditor.setText(preset->getStringAttribute("focusAddress", focusAddressEditor.getText()), juce::dontSendNotification);
        focusInMinEditor.setText(preset->getStringAttribute("focusInMin", focusInMinEditor.getText()), juce::dontSendNotification);
        focusInMaxEditor.setText(preset->getStringAttribute("focusInMax", focusInMaxEditor.getText()), juce::dontSendNotification);
        outAddressEditor.setText(preset->getStringAttribute("outAddress", outAddressEditor.getText()), juce::dontSendNotification);
        inputMinEditor.setText(preset->getStringAttribute("inputMin", inputMinEditor.getText()), juce::dontSendNotification);
        inputMaxEditor.setText(preset->getStringAttribute("inputMax", inputMaxEditor.getText()), juce::dontSendNotification);
        smoothingSlider.setValue(preset->getDoubleAttribute("smoothingMs", smoothingSlider.getValue()), juce::dontSendNotification);
        outputRateSlider.setValue(preset->getDoubleAttribute("outputRateHz", outputRateSlider.getValue()), juce::dontSendNotification);
        restartOutputTimer();
        applyConnections();

        const auto savedLutPath = preset->getStringAttribute("lutFile").trim();
        if (savedLutPath.isNotEmpty())
        {
            const auto lutFile = juce::File(savedLutPath);
            if (lutFile.existsAsFile())
                handleLutFileChosen(lutFile);
        }

        const auto savedSurfacePath = preset->getStringAttribute("surfaceFile").trim();
        if (savedSurfacePath.isNotEmpty())
        {
            const auto surfaceFile = juce::File(savedSurfacePath);
            if (surfaceFile.existsAsFile())
                handleSurfaceFileChosen(surfaceFile);
        }

        settingsXml->setAttribute("lastPreset", presetName);
        juce::String writeError;
        if (!writeSettingsXml(*settingsXml, writeError))
        {
            error = writeError;
            return false;
        }

        return true;
    }

    void loadLastPresetFromSettings()
    {
        auto settingsXml = loadSettingsXml();
        if (settingsXml == nullptr || !settingsXml->hasTagName(kSettingsRootTag))
        {
            settingsStatusLabel.setText("Settings: using defaults", juce::dontSendNotification);
            return;
        }

        const auto presetName = settingsXml->getStringAttribute("lastPreset", "default").trim();
        presetNameEditor.setText(presetName, juce::dontSendNotification);

        juce::String error;
        if (!loadPreset(presetName, error))
        {
            settingsStatusLabel.setText("Settings load failed: " + error, juce::dontSendNotification);
            return;
        }

        settingsStatusLabel.setText("Preset loaded: " + presetName, juce::dontSendNotification);
    }

    void saveCurrentPreset(bool quiet)
    {
        const auto presetName = currentPresetName();
        juce::String error;
        if (!savePreset(presetName, error))
        {
            if (!quiet)
                settingsStatusLabel.setText("Preset save failed: " + error, juce::dontSendNotification);
            return;
        }

        if (!quiet)
            settingsStatusLabel.setText("Preset saved: " + presetName, juce::dontSendNotification);
    }

    void loadCurrentPreset()
    {
        const auto presetName = currentPresetName();
        juce::String error;
        if (!loadPreset(presetName, error))
        {
            settingsStatusLabel.setText("Preset load failed: " + error, juce::dontSendNotification);
            return;
        }

        settingsStatusLabel.setText("Preset loaded: " + presetName, juce::dontSendNotification);
    }

    void saveLutToFile()
    {
        if (lutOutputValues.size() < static_cast<size_t>(kMinLutEntryCount))
        {
            lutStatusLabel.setText("LUT save failed: need at least " + juce::String(kMinLutEntryCount) + " points", juce::dontSendNotification);
            return;
        }

        juce::File defaultTarget = juce::File::getSpecialLocation(juce::File::userDocumentsDirectory).getChildFile("lut_saved.csv");
        if (lastLutFilePath.isNotEmpty())
            defaultTarget = juce::File(lastLutFilePath);

        saveLutFileChooser = std::make_unique<juce::FileChooser>("Save LUT file", defaultTarget, "*.csv");
        const auto chooserFlags = juce::FileBrowserComponent::saveMode
            | juce::FileBrowserComponent::canSelectFiles
            | juce::FileBrowserComponent::warnAboutOverwriting;

        auto safeThis = juce::Component::SafePointer<MainComponent>(this);
        saveLutFileChooser->launchAsync(chooserFlags, [safeThis](const juce::FileChooser& chooser)
        {
            if (safeThis == nullptr)
                return;

            safeThis->handleSaveLutFileChosen(chooser.getResult());
            safeThis->saveLutFileChooser.reset();
        });
    }

    void handleSaveLutFileChosen(const juce::File& file)
    {
        if (file == juce::File {})
            return;

        auto target = file;
        if (!target.hasFileExtension("csv"))
            target = target.withFileExtension(".csv");

        const bool explicitInput = lutInputValues.size() == lutOutputValues.size();
        const auto inMin = static_cast<float>(inputMinEditor.getText().trim().getDoubleValue());
        auto inMax = static_cast<float>(inputMaxEditor.getText().trim().getDoubleValue());
        if (!explicitInput && inMax <= inMin)
            inMax = inMin + 1.0f;

        juce::StringArray lines;
        lines.ensureStorageAllocated(static_cast<int>(lutOutputValues.size()));
        for (size_t i = 0; i < lutOutputValues.size(); ++i)
        {
            const auto t = lutOutputValues.size() <= 1
                ? 0.0f
                : static_cast<float>(i) / static_cast<float>(lutOutputValues.size() - 1);
            const auto x = explicitInput ? lutInputValues[i] : (inMin + t * (inMax - inMin));
            const auto y = lutOutputValues[i];
            lines.add(juce::String(x, 6) + "," + juce::String(y, 6));
        }

        if (!target.replaceWithText(lines.joinIntoString("\n") + "\n"))
        {
            lutStatusLabel.setText("LUT save failed: could not write file", juce::dontSendNotification);
            return;
        }

        lastLutFilePath = target.getFullPathName();
        lutStatusLabel.setText(
            "LUT saved: " + target.getFileName() + " (" + juce::String(static_cast<int>(lutOutputValues.size())) + " points)",
            juce::dontSendNotification);
    }

    std::vector<float> buildNewLutInputPoints(float minValue, float maxValue) const
    {
        constexpr int kFirstSectionPoints = 5;
        constexpr int kSecondSectionPoints = 5;

        const auto span = maxValue - minValue;
        const auto firstSpan = span / 3.0f;
        const auto secondSpan = span - firstSpan;

        std::vector<float> points;
        points.reserve(static_cast<size_t>(2 + kFirstSectionPoints + kSecondSectionPoints));
        points.push_back(minValue);

        for (int i = 1; i <= kFirstSectionPoints; ++i)
        {
            const auto frac = static_cast<float>(i) / static_cast<float>(kFirstSectionPoints + 1);
            points.push_back(minValue + frac * firstSpan);
        }

        const auto secondStart = minValue + firstSpan;
        for (int i = 1; i <= kSecondSectionPoints; ++i)
        {
            const auto frac = static_cast<float>(i) / static_cast<float>(kSecondSectionPoints + 1);
            points.push_back(secondStart + frac * secondSpan);
        }

        points.push_back(maxValue);
        return points;
    }

    std::vector<float> buildNewFocusPoints(float minValue, float maxValue) const
    {
        constexpr int kFocusPointCount = 5;
        std::vector<float> points;
        points.reserve(static_cast<size_t>(kFocusPointCount));
        for (int i = 0; i < kFocusPointCount; ++i)
        {
            const auto t = static_cast<float>(i) / static_cast<float>(kFocusPointCount - 1);
            points.push_back(minValue + t * (maxValue - minValue));
        }
        return points;
    }

    void startNewLutCreation()
    {
        if (newLutCaptureActive)
            return;

        newLutCaptureActive = true;
        newLutHasObservedRaw = false;
        newLutHasObservedFocus = false;
        newLutObservedMin = 0.0f;
        newLutObservedMax = 0.0f;
        newLutObservedFocusMin = 0.0f;
        newLutObservedFocusMax = 0.0f;
        pendingNewLutInputValues.clear();
        pendingNewLutOutputValues.clear();
        pendingNewSurfaceFocusValues.clear();
        pendingNewCreatesSurface = false;
        lutStatusLabel.setText("New LUT: capturing lens + focus min/max, move both now...", juce::dontSendNotification);

        auto safeThis = juce::Component::SafePointer<MainComponent>(this);
        juce::AlertWindow::showOkCancelBox(
            juce::AlertWindow::InfoIcon,
            "Create New LUT",
            "Move lens to MIN/MAX and focus to MIN/MAX, then press OK.",
            "OK",
            "Cancel",
            this,
            juce::ModalCallbackFunction::create([safeThis](int result)
            {
                if (safeThis == nullptr)
                    return;

                safeThis->finishNewLutCreation(result != 0);
            }));
    }

    void finishNewLutCreation(bool confirmed)
    {
        newLutCaptureActive = false;

        if (!confirmed)
        {
            lutStatusLabel.setText("New LUT creation cancelled", juce::dontSendNotification);
            return;
        }

        if (!newLutHasObservedRaw)
        {
            lutStatusLabel.setText("New LUT failed: no input observed", juce::dontSendNotification);
            return;
        }
        if (!newLutHasObservedFocus)
        {
            lutStatusLabel.setText("New LUT failed: no focus observed", juce::dontSendNotification);
            return;
        }

        auto minValue = newLutObservedMin;
        auto maxValue = newLutObservedMax;
        if (maxValue < minValue)
            std::swap(minValue, maxValue);

        if ((maxValue - minValue) <= 1.0e-6f)
        {
            lutStatusLabel.setText("New LUT failed: min/max range too small", juce::dontSendNotification);
            return;
        }

        auto focusMin = newLutObservedFocusMin;
        auto focusMax = newLutObservedFocusMax;
        if (focusMax < focusMin)
            std::swap(focusMin, focusMax);
        if ((focusMax - focusMin) <= 1.0e-6f)
        {
            lutStatusLabel.setText("New LUT failed: focus min/max range too small", juce::dontSendNotification);
            return;
        }

        pendingNewLutInputValues = buildNewLutInputPoints(minValue, maxValue);
        pendingNewLutOutputValues.clear();
        pendingNewLutOutputValues.reserve(pendingNewLutInputValues.size());
        pendingNewSurfaceFocusValues = buildNewFocusPoints(focusMin, focusMax);
        pendingNewCreatesSurface = true;
        const auto denom = pendingNewLutInputValues.size() > 1 ? static_cast<float>(pendingNewLutInputValues.size() - 1) : 1.0f;
        for (size_t i = 0; i < pendingNewLutInputValues.size(); ++i)
        {
            const auto t = static_cast<float>(i) / denom;
            pendingNewLutOutputValues.push_back(
                kNewLutDefaultOutputAtMin + t * (kNewLutDefaultOutputAtMax - kNewLutDefaultOutputAtMin));
        }

        auto defaultFile = juce::File::getSpecialLocation(juce::File::userDocumentsDirectory).getChildFile("surface_new.csv");
        if (lastSurfaceFilePath.isNotEmpty())
        {
            auto previous = juce::File(lastSurfaceFilePath);
            defaultFile = previous.getSiblingFile(previous.getFileNameWithoutExtension() + "_new.csv");
        }
        else if (lastLutFilePath.isNotEmpty())
        {
            auto previous = juce::File(lastLutFilePath);
            defaultFile = previous.getSiblingFile(previous.getFileNameWithoutExtension() + "_surface.csv");
        }

        newLutFileChooser = std::make_unique<juce::FileChooser>("Save New Surface file", defaultFile, "*.csv");
        const auto chooserFlags = juce::FileBrowserComponent::saveMode
            | juce::FileBrowserComponent::canSelectFiles
            | juce::FileBrowserComponent::warnAboutOverwriting;

        auto safeThis = juce::Component::SafePointer<MainComponent>(this);
        newLutFileChooser->launchAsync(chooserFlags, [safeThis](const juce::FileChooser& chooser)
        {
            if (safeThis == nullptr)
                return;

            safeThis->handleNewLutFileChosen(chooser.getResult());
            safeThis->newLutFileChooser.reset();
        });
    }

    void handleNewLutFileChosen(const juce::File& file)
    {
        if (file == juce::File {})
        {
            pendingNewLutInputValues.clear();
            pendingNewLutOutputValues.clear();
            pendingNewSurfaceFocusValues.clear();
            pendingNewCreatesSurface = false;
            lutStatusLabel.setText("New LUT save cancelled", juce::dontSendNotification);
            return;
        }

        const bool hasValidBase =
            pendingNewLutInputValues.size() == pendingNewLutOutputValues.size()
            && pendingNewLutInputValues.size() >= static_cast<size_t>(kMinLutEntryCount);
        const bool hasValidSurfaceFocus = pendingNewSurfaceFocusValues.size() >= 2;
        if (!hasValidBase || (pendingNewCreatesSurface && !hasValidSurfaceFocus))
        {
            pendingNewLutInputValues.clear();
            pendingNewLutOutputValues.clear();
            pendingNewSurfaceFocusValues.clear();
            pendingNewCreatesSurface = false;
            lutStatusLabel.setText("New LUT failed: invalid generated points", juce::dontSendNotification);
            return;
        }

        auto target = file;
        if (!target.hasFileExtension("csv"))
            target = target.withFileExtension(".csv");

        juce::StringArray lines;
        if (pendingNewCreatesSurface)
        {
            lines.ensureStorageAllocated(static_cast<int>(pendingNewLutInputValues.size() * pendingNewSurfaceFocusValues.size()));
            const auto centerIndex = static_cast<int>(pendingNewSurfaceFocusValues.size() / 2);
            for (size_t i = 0; i < pendingNewLutInputValues.size(); ++i)
            {
                for (size_t j = 0; j < pendingNewSurfaceFocusValues.size(); ++j)
                {
                    const auto delta = static_cast<float>(static_cast<int>(j) - centerIndex); // -2,-1,0,+1,+2
                    const auto fov = pendingNewLutOutputValues[i] + delta;
                    lines.add(
                        juce::String(pendingNewLutInputValues[i], 6) + ","
                            + juce::String(pendingNewSurfaceFocusValues[j], 6) + ","
                            + juce::String(fov, 6));
                }
            }
        }
        else
        {
            lines.ensureStorageAllocated(static_cast<int>(pendingNewLutInputValues.size()));
            for (size_t i = 0; i < pendingNewLutInputValues.size(); ++i)
                lines.add(juce::String(pendingNewLutInputValues[i], 6) + "," + juce::String(pendingNewLutOutputValues[i], 6));
        }

        if (!target.replaceWithText(lines.joinIntoString("\n") + "\n"))
        {
            pendingNewLutInputValues.clear();
            pendingNewLutOutputValues.clear();
            pendingNewSurfaceFocusValues.clear();
            pendingNewCreatesSurface = false;
            lutStatusLabel.setText("New LUT save failed: could not write file", juce::dontSendNotification);
            return;
        }

        if (pendingNewCreatesSurface && pendingNewSurfaceFocusValues.size() >= 2)
        {
            focusInMinEditor.setText(juce::String(pendingNewSurfaceFocusValues.front(), 6), juce::dontSendNotification);
            focusInMaxEditor.setText(juce::String(pendingNewSurfaceFocusValues.back(), 6), juce::dontSendNotification);
        }

        const auto createdSurface = pendingNewCreatesSurface;
        pendingNewLutInputValues.clear();
        pendingNewLutOutputValues.clear();
        pendingNewSurfaceFocusValues.clear();
        pendingNewCreatesSurface = false;
        if (createdSurface)
            handleSurfaceFileChosen(target);
        else
            handleLutFileChosen(target);
    }

    void pushLutUndoSnapshot(const LutSnapshot& snapshot)
    {
        lutUndoStack.push_back(snapshot);
        lutRedoStack.clear();
        constexpr size_t kMaxHistory = 256;
        if (lutUndoStack.size() > kMaxHistory)
            lutUndoStack.erase(lutUndoStack.begin());
        updateUndoRedoButtons();
    }

    void refreshPointEditorFromSelection()
    {
        const bool hasSurfaceSelection =
            selectedSurfaceLensIndex >= 0
            && selectedSurfaceFocusIndex >= 0
            && selectedSurfaceLensIndex < static_cast<int>(surfaceLensAxis.size())
            && selectedSurfaceFocusIndex < static_cast<int>(surfaceFocusAxis.size())
            && surfaceValuesGrid.size() == surfaceLensAxis.size() * surfaceFocusAxis.size();

        if (hasSurfaceSelection)
        {
            const auto focusCount = static_cast<int>(surfaceFocusAxis.size());
            const auto flatIndex = static_cast<size_t>(selectedSurfaceLensIndex * focusCount + selectedSurfaceFocusIndex);
            lutCartesianPlot.setSelectedPointIndex(-1);
            lutCartesianPlot.setSelectedSurfacePoint(selectedSurfaceLensIndex, selectedSurfaceFocusIndex);
            pointEditValueEditor.setEnabled(true);
            suppressPointEditorCallbacks = true;
            pointEditValueEditor.setText(juce::String(surfaceValuesGrid[flatIndex], 6), juce::dontSendNotification);
            suppressPointEditorCallbacks = false;
            return;
        }

        if (selectedLutPointIndex < 0 || selectedLutPointIndex >= static_cast<int>(lutOutputValues.size()))
        {
            lutCartesianPlot.setSelectedPointIndex(-1);
            lutCartesianPlot.setSelectedSurfacePoint(-1, -1);
            suppressPointEditorCallbacks = true;
            pointEditValueEditor.setText("-", juce::dontSendNotification);
            suppressPointEditorCallbacks = false;
            pointEditValueEditor.setEnabled(false);
            return;
        }

        lutCartesianPlot.setSelectedPointIndex(selectedLutPointIndex);
        lutCartesianPlot.setSelectedSurfacePoint(-1, -1);
        pointEditValueEditor.setEnabled(true);
        suppressPointEditorCallbacks = true;
        pointEditValueEditor.setText(juce::String(lutOutputValues[static_cast<size_t>(selectedLutPointIndex)], 6), juce::dontSendNotification);
        suppressPointEditorCallbacks = false;
    }

    void selectLutPointForEditing(size_t index)
    {
        if (index >= lutOutputValues.size())
            return;

        selectedSurfaceLensIndex = -1;
        selectedSurfaceFocusIndex = -1;
        selectedLutPointIndex = static_cast<int>(index);
        refreshPointEditorFromSelection();
        lutStatusLabel.setText("Selected LUT point " + juce::String(static_cast<int>(index)), juce::dontSendNotification);
        pointEditValueEditor.grabKeyboardFocus();
    }

    void selectSurfacePointForEditing(size_t lensIndex, size_t focusIndex)
    {
        if (lensIndex >= surfaceLensAxis.size() || focusIndex >= surfaceFocusAxis.size())
            return;

        if (surfaceValuesGrid.size() != surfaceLensAxis.size() * surfaceFocusAxis.size())
            return;

        selectedLutPointIndex = -1;
        selectedSurfaceLensIndex = static_cast<int>(lensIndex);
        selectedSurfaceFocusIndex = static_cast<int>(focusIndex);
        refreshPointEditorFromSelection();
        lutStatusLabel.setText(
            "Selected surface point lens=" + juce::String(surfaceLensAxis[lensIndex], 4)
                + ", focus=" + juce::String(surfaceFocusAxis[focusIndex], 4),
            juce::dontSendNotification);
        pointEditValueEditor.grabKeyboardFocus();
    }

    bool applySelectedLutPointValue(float value)
    {
        if (selectedLutPointIndex < 0 || selectedLutPointIndex >= static_cast<int>(lutOutputValues.size()))
            return false;

        const auto before = captureLutSnapshot();
        handleLutPointDragged(static_cast<size_t>(selectedLutPointIndex), value);
        const auto after = captureLutSnapshot();
        if (lutSnapshotsEqual(before, after))
            return false;

        pushLutUndoSnapshot(before);
        refreshPointEditorFromSelection();
        return true;
    }

    bool applySelectedSurfacePointValue(float value)
    {
        if (selectedSurfaceLensIndex < 0 || selectedSurfaceFocusIndex < 0)
            return false;

        if (selectedSurfaceLensIndex >= static_cast<int>(surfaceLensAxis.size())
            || selectedSurfaceFocusIndex >= static_cast<int>(surfaceFocusAxis.size()))
        {
            return false;
        }

        if (surfaceValuesGrid.size() != surfaceLensAxis.size() * surfaceFocusAxis.size())
            return false;

        const auto focusCount = static_cast<int>(surfaceFocusAxis.size());
        const auto flatIndex = static_cast<size_t>(selectedSurfaceLensIndex * focusCount + selectedSurfaceFocusIndex);
        const auto before = surfaceValuesGrid[flatIndex];
        handleSurfacePointDragged(
            static_cast<size_t>(selectedSurfaceLensIndex),
            static_cast<size_t>(selectedSurfaceFocusIndex),
            value);
        return std::abs(surfaceValuesGrid[flatIndex] - before) > 1.0e-6f;
    }

    bool applySelectedPointValue(float value)
    {
        if (selectedSurfaceLensIndex >= 0 && selectedSurfaceFocusIndex >= 0)
            return applySelectedSurfacePointValue(value);

        return applySelectedLutPointValue(value);
    }

    void handlePointEditorTextChange()
    {
        if (suppressPointEditorCallbacks)
            return;

        float parsedValue = 0.0f;
        if (!parseNumericToken(pointEditValueEditor.getText().trim(), parsedValue))
            return;

        applySelectedPointValue(parsedValue);
    }

    void nudgeSelectedPoint(float delta)
    {
        const bool hasSurfaceSelection =
            selectedSurfaceLensIndex >= 0
            && selectedSurfaceFocusIndex >= 0
            && selectedSurfaceLensIndex < static_cast<int>(surfaceLensAxis.size())
            && selectedSurfaceFocusIndex < static_cast<int>(surfaceFocusAxis.size())
            && surfaceValuesGrid.size() == surfaceLensAxis.size() * surfaceFocusAxis.size();
        const bool hasLutSelection =
            selectedLutPointIndex >= 0
            && selectedLutPointIndex < static_cast<int>(lutOutputValues.size());

        if (!hasSurfaceSelection && !hasLutSelection)
            return;

        float baseValue = 0.0f;
        if (hasSurfaceSelection)
        {
            const auto focusCount = static_cast<int>(surfaceFocusAxis.size());
            const auto flatIndex = static_cast<size_t>(selectedSurfaceLensIndex * focusCount + selectedSurfaceFocusIndex);
            baseValue = surfaceValuesGrid[flatIndex];
        }
        else
        {
            baseValue = lutOutputValues[static_cast<size_t>(selectedLutPointIndex)];
        }

        float parsedValue = 0.0f;
        if (parseNumericToken(pointEditValueEditor.getText().trim(), parsedValue))
            baseValue = parsedValue;

        const auto nextValue = baseValue + delta;
        suppressPointEditorCallbacks = true;
        pointEditValueEditor.setText(juce::String(nextValue, 6), juce::dontSendNotification);
        suppressPointEditorCallbacks = false;
        applySelectedPointValue(nextValue);
    }

    void addLutPointAtCurrentInput()
    {
        if (lutOutputValues.empty())
        {
            lutStatusLabel.setText("Add point failed: load/create LUT first", juce::dontSendNotification);
            return;
        }

        const auto before = captureLutSnapshot();
        const bool hadExplicitInput = (lutInputValues.size() == lutOutputValues.size());

        if (!hadExplicitInput)
        {
            const auto inMin = inputMinEditor.getText().trim().getDoubleValue();
            const auto inMax = inputMaxEditor.getText().trim().getDoubleValue();
            if (inMax <= inMin)
            {
                lutStatusLabel.setText("Add point failed: invalid Input Min/Max", juce::dontSendNotification);
                return;
            }

            lutInputValues.resize(lutOutputValues.size());
            for (size_t i = 0; i < lutOutputValues.size(); ++i)
            {
                const auto t = lutOutputValues.size() <= 1
                    ? 0.0
                    : static_cast<double>(i) / static_cast<double>(lutOutputValues.size() - 1);
                lutInputValues[i] = static_cast<float>(inMin + t * (inMax - inMin));
            }
        }

        const auto x = rawValue.load(std::memory_order_relaxed);
        const auto y = applyLutCalibration(x);

        constexpr float kEpsilon = 1.0e-6f;
        auto it = std::lower_bound(lutInputValues.begin(), lutInputValues.end(), x);
        auto insertIdx = static_cast<size_t>(std::distance(lutInputValues.begin(), it));

        if (it != lutInputValues.end() && std::abs(*it - x) <= kEpsilon)
        {
            lutOutputValues[insertIdx] = y;
        }
        else if (insertIdx > 0 && std::abs(lutInputValues[insertIdx - 1] - x) <= kEpsilon)
        {
            --insertIdx;
            lutOutputValues[insertIdx] = y;
        }
        else
        {
            const auto pos = lutInputValues.begin() + static_cast<std::vector<float>::difference_type>(insertIdx);
            lutInputValues.insert(pos, x);
            lutOutputValues.insert(lutOutputValues.begin() + static_cast<std::vector<float>::difference_type>(insertIdx), y);
        }

        inputMinEditor.setText(juce::String(lutInputValues.front(), 6), juce::dontSendNotification);
        inputMaxEditor.setText(juce::String(lutInputValues.back(), 6), juce::dontSendNotification);

#if LININTERPOL_HAVE_ALGLIB
        juce::String splineError;
        if (!rebuildLutSpline(splineError))
        {
            juce::String restoreError;
            restoreLutSnapshot(before, restoreError);
            lutStatusLabel.setText("Add point failed: " + splineError, juce::dontSendNotification);
            return;
        }
#endif

        lutCartesianPlot.setLutValues(lutOutputValues, lutInputValues, true);
        const auto inMin = inputMinEditor.getText().trim().getDoubleValue();
        auto inMax = inputMaxEditor.getText().trim().getDoubleValue();
        if (inMax <= inMin)
            inMax = inMin + 1.0;
        rebuildCubicCurvePreview(inMin, inMax);
        lastCurveInMin = inMin;
        lastCurveInMax = inMax;
        cubicCurveDirty = false;

        selectedLutPointIndex = static_cast<int>(insertIdx);
        refreshPointEditorFromSelection();

        const auto after = captureLutSnapshot();
        if (!lutSnapshotsEqual(before, after))
            pushLutUndoSnapshot(before);

        lutStatusLabel.setText(
            "Added LUT point at input " + juce::String(x, 4) + " (point " + juce::String(static_cast<int>(insertIdx)) + ")",
            juce::dontSendNotification);
    }

    LutSnapshot captureLutSnapshot() const
    {
        LutSnapshot snapshot;
        snapshot.inputValues = lutInputValues;
        snapshot.outputValues = lutOutputValues;
        return snapshot;
    }

    static bool lutSnapshotsEqual(const LutSnapshot& a, const LutSnapshot& b)
    {
        if (a.inputValues.size() != b.inputValues.size() || a.outputValues.size() != b.outputValues.size())
            return false;

        constexpr float kEpsilon = 1.0e-6f;
        for (size_t i = 0; i < a.inputValues.size(); ++i)
        {
            if (std::abs(a.inputValues[i] - b.inputValues[i]) > kEpsilon)
                return false;
        }
        for (size_t i = 0; i < a.outputValues.size(); ++i)
        {
            if (std::abs(a.outputValues[i] - b.outputValues[i]) > kEpsilon)
                return false;
        }
        return true;
    }

    void clearLutHistory()
    {
        lutUndoStack.clear();
        lutRedoStack.clear();
        dragHistoryActive = false;
        updateUndoRedoButtons();
    }

    void updateUndoRedoButtons()
    {
        undoLutButton.setEnabled(!lutUndoStack.empty());
        redoLutButton.setEnabled(!lutRedoStack.empty());
    }

    void beginLutDragHistory()
    {
        if (lutOutputValues.size() < static_cast<size_t>(kMinLutEntryCount))
            return;

        dragStartSnapshot = captureLutSnapshot();
        dragHistoryActive = true;
    }

    void endLutDragHistory()
    {
        if (!dragHistoryActive)
            return;

        dragHistoryActive = false;
        const auto current = captureLutSnapshot();
        if (lutSnapshotsEqual(dragStartSnapshot, current))
            return;

        pushLutUndoSnapshot(dragStartSnapshot);
    }

    bool restoreLutSnapshot(const LutSnapshot& snapshot, juce::String& error)
    {
        if (snapshot.outputValues.size() < static_cast<size_t>(kMinLutEntryCount))
        {
            error = "Snapshot has fewer than " + juce::String(kMinLutEntryCount) + " points";
            return false;
        }

        lutInputValues = snapshot.inputValues;
        lutOutputValues = snapshot.outputValues;

        if (!lutInputValues.empty())
        {
            inputMinEditor.setText(juce::String(lutInputValues.front(), 6), juce::dontSendNotification);
            inputMaxEditor.setText(juce::String(lutInputValues.back(), 6), juce::dontSendNotification);
        }

#if LININTERPOL_HAVE_ALGLIB
        if (!rebuildLutSpline(error))
            return false;
#endif

        lutCartesianPlot.setLutValues(lutOutputValues, lutInputValues);
        const auto inMin = inputMinEditor.getText().trim().getDoubleValue();
        auto inMax = inputMaxEditor.getText().trim().getDoubleValue();
        if (inMax <= inMin)
            inMax = inMin + 1.0;
        rebuildCubicCurvePreview(inMin, inMax);
        lastCurveInMin = inMin;
        lastCurveInMax = inMax;
        cubicCurveDirty = false;
        if (selectedLutPointIndex >= static_cast<int>(lutOutputValues.size()))
            selectedLutPointIndex = -1;
        refreshPointEditorFromSelection();
        return true;
    }

    void undoLutChange()
    {
        if (lutUndoStack.empty())
        {
            lutStatusLabel.setText("Undo: nothing to undo", juce::dontSendNotification);
            return;
        }

        const auto current = captureLutSnapshot();
        const auto previous = lutUndoStack.back();
        juce::String error;
        if (!restoreLutSnapshot(previous, error))
        {
            lutStatusLabel.setText("Undo failed: " + error, juce::dontSendNotification);
            return;
        }

        lutUndoStack.pop_back();
        lutRedoStack.push_back(current);
        updateUndoRedoButtons();
        lutStatusLabel.setText("Undo LUT change", juce::dontSendNotification);
    }

    void redoLutChange()
    {
        if (lutRedoStack.empty())
        {
            lutStatusLabel.setText("Redo: nothing to redo", juce::dontSendNotification);
            return;
        }

        const auto current = captureLutSnapshot();
        const auto next = lutRedoStack.back();
        juce::String error;
        if (!restoreLutSnapshot(next, error))
        {
            lutStatusLabel.setText("Redo failed: " + error, juce::dontSendNotification);
            return;
        }

        lutRedoStack.pop_back();
        lutUndoStack.push_back(current);
        updateUndoRedoButtons();
        lutStatusLabel.setText("Redo LUT change", juce::dontSendNotification);
    }

#if LININTERPOL_HAVE_ALGLIB
    bool rebuildSurfaceSpline(juce::String& error)
    {
        if (surfaceLensAxis.size() < static_cast<size_t>(kMinSurfaceAxisCount)
            || surfaceFocusAxis.size() < static_cast<size_t>(kMinSurfaceAxisCount))
        {
            surfaceSplineReady = false;
            error = "Surface needs at least " + juce::String(kMinSurfaceAxisCount) + "x" + juce::String(kMinSurfaceAxisCount) + " axes";
            return false;
        }

        const auto lensCount = static_cast<int>(surfaceLensAxis.size());   // X axis size -> N in ALGLIB
        const auto focusCount = static_cast<int>(surfaceFocusAxis.size()); // Y axis size -> M in ALGLIB
        if (surfaceValuesGrid.size() != static_cast<size_t>(lensCount * focusCount))
        {
            surfaceSplineReady = false;
            error = "Surface grid size mismatch";
            return false;
        }

        alglib::real_1d_array lens;
        alglib::real_1d_array focus;
        alglib::real_2d_array f;
        lens.setlength(lensCount);
        focus.setlength(focusCount);
        f.setlength(focusCount, lensCount); // rows=M(y), cols=N(x)

        for (int i = 0; i < lensCount; ++i)
            lens[i] = static_cast<double>(surfaceLensAxis[static_cast<size_t>(i)]);
        for (int j = 0; j < focusCount; ++j)
            focus[j] = static_cast<double>(surfaceFocusAxis[static_cast<size_t>(j)]);
        for (int j = 0; j < focusCount; ++j)
        {
            for (int i = 0; i < lensCount; ++i)
            {
                // source grid is lens-major (i*focusCount + j), ALGLIB expects F[y][x] = F[M][N]
                f[j][i] = static_cast<double>(surfaceValuesGrid[static_cast<size_t>(i * focusCount + j)]);
            }
        }

        try
        {
            alglib::spline2dbuildbicubic(lens, focus, f, focusCount, lensCount, surfaceSpline);
            surfaceLensMin = static_cast<double>(surfaceLensAxis.front());
            surfaceLensMax = static_cast<double>(surfaceLensAxis.back());
            surfaceFocusMin = static_cast<double>(surfaceFocusAxis.front());
            surfaceFocusMax = static_cast<double>(surfaceFocusAxis.back());
            surfaceSplineReady = true;
            return true;
        }
        catch (const alglib::ap_error& ex)
        {
            surfaceSplineReady = false;
            error = juce::String("ALGLIB 2D error: ") + juce::String(ex.msg.c_str());
            return false;
        }
    }

    bool rebuildLutSpline(juce::String& error)
    {
        if (lutOutputValues.size() < static_cast<size_t>(kMinLutEntryCount))
        {
            splineReady = false;
            error = "Need at least " + juce::String(kMinLutEntryCount) + " LUT values for cubic spline";
            return false;
        }

        alglib::real_1d_array x;
        alglib::real_1d_array y;
        x.setlength(static_cast<int>(lutOutputValues.size()));
        y.setlength(static_cast<int>(lutOutputValues.size()));

        const bool explicitInput = lutInputValues.size() == lutOutputValues.size();
        for (size_t i = 0; i < lutOutputValues.size(); ++i)
        {
            const auto t = static_cast<double>(i) / static_cast<double>(lutOutputValues.size() - 1);
            x[static_cast<int>(i)] = explicitInput ? static_cast<double>(lutInputValues[i]) : t;
            y[static_cast<int>(i)] = static_cast<double>(lutOutputValues[i]);
        }

        try
        {
            alglib::spline1dbuildcubic(x, y, lutSpline);
            splineUsesExplicitInputAxis = explicitInput;
            if (explicitInput)
            {
                splineXMin = static_cast<double>(lutInputValues.front());
                splineXMax = static_cast<double>(lutInputValues.back());
            }
            splineReady = true;
            return true;
        }
        catch (const alglib::ap_error& ex)
        {
            splineReady = false;
            error = juce::String("ALGLIB error: ") + juce::String(ex.msg.c_str());
            return false;
        }
    }
#endif

    void rebuildCubicCurvePreview(double inMin, double inMax)
    {
        cubicCurveX.clear();
        cubicCurveY.clear();

        if (lutOutputValues.size() < static_cast<size_t>(kMinLutEntryCount))
        {
            lutCartesianPlot.setCubicCurve(cubicCurveX, cubicCurveY);
            return;
        }

        constexpr int kCurvePoints = 240;
        cubicCurveX.reserve(static_cast<size_t>(kCurvePoints));
        cubicCurveY.reserve(static_cast<size_t>(kCurvePoints));

        const bool explicitInput = lutInputValues.size() == lutOutputValues.size();
        double xStart = inMin;
        double xEnd = inMax;

        if (explicitInput)
        {
            xStart = static_cast<double>(lutInputValues.front());
            xEnd = static_cast<double>(lutInputValues.back());
        }

        if (xEnd <= xStart)
        {
            lutCartesianPlot.setCubicCurve(cubicCurveX, cubicCurveY);
            return;
        }

        const auto previewFocus = getSplinePreviewFocusValue();
        for (int i = 0; i < kCurvePoints; ++i)
        {
            const auto t = static_cast<double>(i) / static_cast<double>(kCurvePoints - 1);
            const auto x = xStart + t * (xEnd - xStart);
            cubicCurveX.push_back(static_cast<float>(x));
            cubicCurveY.push_back(applyLutCalibration(static_cast<float>(x), previewFocus));
        }

        lutCartesianPlot.setCubicCurve(cubicCurveX, cubicCurveY);
    }

    size_t getBaseFocusIndex() const
    {
        if (surfaceFocusAxis.empty())
            return 0;

        constexpr float kBaseFocus = 0.5f; // ±0 focus slice
        auto bestIdx = size_t { 0 };
        auto bestDist = std::abs(surfaceFocusAxis[0] - kBaseFocus);
        for (size_t i = 1; i < surfaceFocusAxis.size(); ++i)
        {
            const auto d = std::abs(surfaceFocusAxis[i] - kBaseFocus);
            if (d < bestDist)
            {
                bestDist = d;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    float getSplinePreviewFocusValue() const
    {
        if (surfaceCalibrationActive() && !surfaceFocusAxis.empty())
            return surfaceFocusAxis[getBaseFocusIndex()];

        return focusValue.load(std::memory_order_relaxed);
    }

    void handleLutPointDragged(size_t index, float outputValue)
    {
        if (index >= lutOutputValues.size())
            return;

        const auto previousValue = lutOutputValues[index];
        lutOutputValues[index] = outputValue;

#if LININTERPOL_HAVE_ALGLIB
        juce::String splineError;
        if (!rebuildLutSpline(splineError))
        {
            lutOutputValues[index] = previousValue;
            lutCartesianPlot.setLutValues(lutOutputValues, lutInputValues, true);
            lutStatusLabel.setText("LUT point edit failed: " + splineError, juce::dontSendNotification);
            return;
        }
#endif

        lutCartesianPlot.setLutValues(lutOutputValues, lutInputValues, true);

        const auto inMin = inputMinEditor.getText().trim().getDoubleValue();
        auto inMax = inputMaxEditor.getText().trim().getDoubleValue();
        if (inMax <= inMin)
            inMax = inMin + 1.0;
        rebuildCubicCurvePreview(inMin, inMax);
        lastCurveInMin = inMin;
        lastCurveInMax = inMax;
        cubicCurveDirty = false;

        lutStatusLabel.setText(
            "LUT point " + juce::String(static_cast<int>(index)) + " = " + juce::String(outputValue, 4),
            juce::dontSendNotification);

        if (selectedLutPointIndex == static_cast<int>(index))
            refreshPointEditorFromSelection();
    }

    void handleSurfacePointDragged(size_t lensIndex, size_t focusIndex, float outputValue)
    {
        if (lensIndex >= surfaceLensAxis.size() || focusIndex >= surfaceFocusAxis.size())
            return;

        const auto focusCount = surfaceFocusAxis.size();
        const auto flatIndex = lensIndex * focusCount + focusIndex;
        if (flatIndex >= surfaceValuesGrid.size())
            return;

        constexpr float kEpsilon = 1.0e-6f;
        const auto previousValue = surfaceValuesGrid[flatIndex];
        const auto baseFocusIndex = getBaseFocusIndex();
        const bool movingBasePoint = focusIndex == baseFocusIndex;
        const auto rowStart = lensIndex * focusCount;
        std::vector<float> previousRowValues;

        if (movingBasePoint)
        {
            const auto delta = outputValue - previousValue;
            if (std::abs(delta) <= kEpsilon)
                return;

            previousRowValues.reserve(focusCount);
            for (size_t j = 0; j < focusCount; ++j)
            {
                const auto idx = rowStart + j;
                previousRowValues.push_back(surfaceValuesGrid[idx]);
                surfaceValuesGrid[idx] += delta;
            }
        }
        else
        {
            if (std::abs(previousValue - outputValue) <= kEpsilon)
                return;

            surfaceValuesGrid[flatIndex] = outputValue;
        }

#if LININTERPOL_HAVE_ALGLIB
        juce::String splineError;
        if (!rebuildSurfaceSpline(splineError))
        {
            if (movingBasePoint)
            {
                for (size_t j = 0; j < focusCount; ++j)
                    surfaceValuesGrid[rowStart + j] = previousRowValues[j];
            }
            else
            {
                surfaceValuesGrid[flatIndex] = previousValue;
            }
            lutCartesianPlot.setSurfaceValues(surfaceLensAxis, surfaceFocusAxis, surfaceValuesGrid);
            lutStatusLabel.setText("Surface point edit failed: " + splineError, juce::dontSendNotification);
            return;
        }
#endif

        lutCartesianPlot.setSurfaceValues(surfaceLensAxis, surfaceFocusAxis, surfaceValuesGrid);

        const auto inMin = inputMinEditor.getText().trim().getDoubleValue();
        auto inMax = inputMaxEditor.getText().trim().getDoubleValue();
        if (inMax <= inMin)
            inMax = inMin + 1.0;
        rebuildCubicCurvePreview(inMin, inMax);
        lastCurveInMin = inMin;
        lastCurveInMax = inMax;
        cubicCurveDirty = false;

        if (movingBasePoint)
        {
            lutStatusLabel.setText(
                "Surface base moved at lens=" + juce::String(surfaceLensAxis[lensIndex], 4)
                    + " -> all focus points shifted",
                juce::dontSendNotification);
        }
        else
        {
            lutStatusLabel.setText(
                "Surface point lens=" + juce::String(surfaceLensAxis[lensIndex], 4)
                    + ", focus=" + juce::String(surfaceFocusAxis[focusIndex], 4)
                    + " -> " + juce::String(outputValue, 4),
                juce::dontSendNotification);
        }

        selectedLutPointIndex = -1;
        selectedSurfaceLensIndex = static_cast<int>(lensIndex);
        selectedSurfaceFocusIndex = static_cast<int>(focusIndex);
        refreshPointEditorFromSelection();
    }

    void oscMessageReceived(const juce::OSCMessage& message) override
    {
        bool ok = false;
        const auto value = parseFirstOscValue(message, ok);
        if (!ok)
            return;

        const auto address = message.getAddressPattern().toString();
        if (address == inAddressEditor.getText().trim())
        {
            rawValue.store(value, std::memory_order_relaxed);
            receivedCount.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        if (address == focusAddressEditor.getText().trim())
        {
            focusValue.store(value, std::memory_order_relaxed);
            receivedCount.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void timerCallback() override
    {
        pollFreeDInput();
        const auto nowSeconds = juce::Time::getMillisecondCounterHiRes() * 0.001;
        auto dt = nowSeconds - lastTickSeconds;
        if (lastTickSeconds <= 0.0 || dt <= 0.0 || dt > 1.0)
            dt = 1.0 / juce::jmax(1.0, outputRateSlider.getValue());
        lastTickSeconds = nowSeconds;

        const auto raw = rawValue.load(std::memory_order_relaxed);
        const auto focus = focusValue.load(std::memory_order_relaxed);
        if (newLutCaptureActive)
        {
            if (!newLutHasObservedRaw)
            {
                newLutObservedMin = raw;
                newLutObservedMax = raw;
                newLutHasObservedRaw = true;
            }
            else
            {
                newLutObservedMin = juce::jmin(newLutObservedMin, raw);
                newLutObservedMax = juce::jmax(newLutObservedMax, raw);
            }

            if (!newLutHasObservedFocus)
            {
                newLutObservedFocusMin = focus;
                newLutObservedFocusMax = focus;
                newLutHasObservedFocus = true;
            }
            else
            {
                newLutObservedFocusMin = juce::jmin(newLutObservedFocusMin, focus);
                newLutObservedFocusMax = juce::jmax(newLutObservedFocusMax, focus);
            }
        }
        const auto target2D = applyLutCalibration(raw, focus);
        const auto lutSample = surfaceCalibrationActive() ? target2D : getNearestLutSample(raw);
        const auto mappedFocus = mapFocusInputToSurfaceDomain(focus);

        const auto yMin = static_cast<float>(inputMinEditor.getText().trim().getDoubleValue());
        auto yMax = static_cast<float>(inputMaxEditor.getText().trim().getDoubleValue());
        if (yMax <= yMin)
            yMax = yMin + 1.0f;
        historyPlot.setYRange(yMin, yMax);
        lutCartesianPlot.setInputRange(yMin, yMax);
        const auto inMin = static_cast<double>(yMin);
        const auto inMax = static_cast<double>(yMax);
        if (cubicCurveDirty || std::abs(inMin - lastCurveInMin) > 1.0e-9 || std::abs(inMax - lastCurveInMax) > 1.0e-9)
        {
            rebuildCubicCurvePreview(inMin, inMax);
            lastCurveInMin = inMin;
            lastCurveInMax = inMax;
            cubicCurveDirty = false;
        }

        const auto smoothMs = smoothingSlider.getValue();
        const float alpha = smoothMs <= 0.0
            ? 1.0f
            : static_cast<float>(1.0 - std::exp(-(dt / (smoothMs * 0.001))));

        interpolatedValue += alpha * (target2D - interpolatedValue);
        historyPlot.pushSample(raw, interpolatedValue);
        lutCartesianPlot.setPoints(raw, lutSample, target2D);

        if (outputConnected && outAddressEditor.getText().trim().isNotEmpty())
            sender.send(juce::OSCMessage(outAddressEditor.getText().trim(), interpolatedValue));

        rawValueLabel.setText("Raw: " + juce::String(raw, 5), juce::dontSendNotification);
        if (surfaceCalibrationActive())
        {
            focusValueLabel.setText(
                "Focus: " + juce::String(focus, 5) + " -> " + juce::String(mappedFocus, 5),
                juce::dontSendNotification);
        }
        else
        {
            focusValueLabel.setText("Focus: " + juce::String(focus, 5), juce::dontSendNotification);
        }
        if (!freeDEnableToggle.getToggleState())
        {
            freeDStatusLabel.setText("FreeD: disabled", juce::dontSendNotification);
        }
        else if (freeDConnected)
        {
            freeDStatusLabel.setText(
                "FreeD: cam " + juce::String(freeDLastCameraId)
                    + " zoom " + juce::String(freeDLastZoom)
                    + " focus " + juce::String(freeDLastFocus)
                    + " packets " + juce::String(static_cast<int>(freeDPacketsReceived)),
                juce::dontSendNotification);
        }
        else
        {
            freeDStatusLabel.setText("FreeD: listen failed", juce::dontSendNotification);
        }
        targetValueLabel.setText("Target: " + juce::String(target2D, 5), juce::dontSendNotification);
        interpValueLabel.setText("Interpolated: " + juce::String(interpolatedValue, 5), juce::dontSendNotification);
        receivedCountLabel.setText("Received Messages: " + juce::String(receivedCount.load(std::memory_order_relaxed)), juce::dontSendNotification);
    }

    void loadSurfaceFromFile()
    {
        surfaceFileChooser = std::make_unique<juce::FileChooser>("Load 2D surface file", juce::File {}, "*.csv;*.txt");

        const auto chooserFlags = juce::FileBrowserComponent::openMode
            | juce::FileBrowserComponent::canSelectFiles;

        auto safeThis = juce::Component::SafePointer<MainComponent>(this);
        surfaceFileChooser->launchAsync(chooserFlags, [safeThis](const juce::FileChooser& chooser)
        {
            if (safeThis == nullptr)
                return;

            safeThis->handleSurfaceFileChosen(chooser.getResult());
            safeThis->surfaceFileChooser.reset();
        });
    }

    void handleSurfaceFileChosen(const juce::File& file)
    {
        if (!file.existsAsFile())
            return;

        ParsedSurfaceData parsed;
        juce::String error;
        if (!parseSurfaceText(file.loadFileAsString(), parsed, error))
        {
            lutStatusLabel.setText("Surface load failed: " + error, juce::dontSendNotification);
            return;
        }

        surfaceLensAxis = std::move(parsed.lensAxis);
        surfaceFocusAxis = std::move(parsed.focusAxis);
        surfaceValuesGrid = std::move(parsed.values);
        if (!surfaceLensAxis.empty())
        {
            inputMinEditor.setText(juce::String(surfaceLensAxis.front(), 6), juce::dontSendNotification);
            inputMaxEditor.setText(juce::String(surfaceLensAxis.back(), 6), juce::dontSendNotification);
        }

#if LININTERPOL_HAVE_ALGLIB
        juce::String splineError;
        if (!rebuildSurfaceSpline(splineError))
        {
            lutStatusLabel.setText("Surface load failed: " + splineError, juce::dontSendNotification);
            return;
        }
#endif

        lutCartesianPlot.setSurfaceValues(surfaceLensAxis, surfaceFocusAxis, surfaceValuesGrid);
        selectedLutPointIndex = -1;
        selectedSurfaceLensIndex = -1;
        selectedSurfaceFocusIndex = -1;
        refreshPointEditorFromSelection();
        lastSurfaceFilePath = file.getFullPathName();
        cubicCurveDirty = true;
        lutStatusLabel.setText(
            "Surface loaded: " + file.getFileName()
                + " (" + juce::String(static_cast<int>(surfaceLensAxis.size())) + " lens x "
                + juce::String(static_cast<int>(surfaceFocusAxis.size())) + " focus)"
                + " | 2D bicubic active",
            juce::dontSendNotification);
    }

    void loadLutFromFile()
    {
        lutFileChooser = std::make_unique<juce::FileChooser>("Load LUT file", juce::File {}, "*.txt;*.csv");

        const auto chooserFlags = juce::FileBrowserComponent::openMode
            | juce::FileBrowserComponent::canSelectFiles;

        auto safeThis = juce::Component::SafePointer<MainComponent>(this);
        lutFileChooser->launchAsync(chooserFlags, [safeThis](const juce::FileChooser& chooser)
        {
            if (safeThis == nullptr)
                return;

            safeThis->handleLutFileChosen(chooser.getResult());
            safeThis->lutFileChooser.reset();
        });
    }

    void handleLutFileChosen(const juce::File& file)
    {
        if (!file.existsAsFile())
            return;

        lastLutFilePath = file.getFullPathName();

        ParsedLutData parsed;
        juce::String error;
        if (!parseLutText(file.loadFileAsString(), parsed, error))
        {
            lutStatusLabel.setText("LUT load failed: " + error, juce::dontSendNotification);
            return;
        }

        if (parsed.hasExplicitInput)
        {
            std::vector<std::pair<float, float>> pairs;
            pairs.reserve(parsed.outputValues.size());
            for (size_t i = 0; i < parsed.outputValues.size(); ++i)
                pairs.emplace_back(parsed.inputValues[i], parsed.outputValues[i]);

            std::sort(pairs.begin(), pairs.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

            lutInputValues.clear();
            lutOutputValues.clear();
            lutInputValues.reserve(pairs.size());
            lutOutputValues.reserve(pairs.size());

            constexpr float kEpsilon = 1.0e-6f;
            for (const auto& p : pairs)
            {
                if (!lutInputValues.empty() && std::abs(p.first - lutInputValues.back()) <= kEpsilon)
                {
                    lutOutputValues.back() = p.second; // keep latest Y for duplicate X
                }
                else
                {
                    lutInputValues.push_back(p.first);
                    lutOutputValues.push_back(p.second);
                }
            }
        }
        else
        {
            lutInputValues.clear();
            lutOutputValues = std::move(parsed.outputValues);
        }

        if (lutOutputValues.size() < static_cast<size_t>(kMinLutEntryCount))
        {
            lutStatusLabel.setText("LUT load failed: need at least " + juce::String(kMinLutEntryCount) + " unique points", juce::dontSendNotification);
            return;
        }

        if (!lutInputValues.empty())
        {
            inputMinEditor.setText(juce::String(lutInputValues.front(), 6), juce::dontSendNotification);
            inputMaxEditor.setText(juce::String(lutInputValues.back(), 6), juce::dontSendNotification);
        }

        cubicCurveDirty = true;

#if LININTERPOL_HAVE_ALGLIB
        auto interpolationMode = juce::String("Cubic spline active (ALGLIB)");
        juce::String splineError;
        if (!rebuildLutSpline(splineError))
        {
            lutStatusLabel.setText("LUT loaded but spline failed: " + splineError, juce::dontSendNotification);
            lutCartesianPlot.setLutValues(lutOutputValues, lutInputValues);
            return;
        }
#else
        auto interpolationMode = juce::String("ALGLIB disabled, using linear interpolation");
#endif

        lastLutFilePath = file.getFullPathName();
        lutCartesianPlot.setLutValues(lutOutputValues, lutInputValues);
        clearLutHistory();
        selectedLutPointIndex = -1;
        refreshPointEditorFromSelection();
        lutStatusLabel.setText(
            "LUT loaded: " + file.getFileName()
                + " (" + juce::String(static_cast<int>(lutOutputValues.size())) + " points)"
                + " | " + interpolationMode
                + (surfaceCalibrationActive() ? " | 2D surface active" : ""),
            juce::dontSendNotification);
    }

    bool surfaceCalibrationActive() const
    {
#if LININTERPOL_HAVE_ALGLIB
        return surfaceSplineReady;
#else
        return false;
#endif
    }

    float mapFocusInputToSurfaceDomain(float focusInput) const
    {
        if (surfaceFocusAxis.empty())
            return focusInput;

        const auto inMin = focusInMinEditor.getText().trim().getDoubleValue();
        const auto inMax = focusInMaxEditor.getText().trim().getDoubleValue();
        if (inMax <= inMin)
            return focusInput;

        const auto t = juce::jlimit(0.0, 1.0, (static_cast<double>(focusInput) - inMin) / (inMax - inMin));
        return static_cast<float>(static_cast<double>(surfaceFocusAxis.front())
            + t * static_cast<double>(surfaceFocusAxis.back() - surfaceFocusAxis.front()));
    }

    float applyLutCalibration(float rawInput) const
    {
        return applyLutCalibration(rawInput, focusValue.load(std::memory_order_relaxed));
    }

    float applyLutCalibration(float rawInput, float focusInput) const
    {
        if (surfaceCalibrationActive())
        {
#if LININTERPOL_HAVE_ALGLIB
            const auto x = juce::jlimit(surfaceLensMin, surfaceLensMax, static_cast<double>(rawInput));
            const auto mappedFocus = mapFocusInputToSurfaceDomain(focusInput);
            const auto y = juce::jlimit(surfaceFocusMin, surfaceFocusMax, static_cast<double>(mappedFocus));
            return static_cast<float>(alglib::spline2dcalc(surfaceSpline, x, y));
#endif
        }

        if (lutOutputValues.size() < static_cast<size_t>(kMinLutEntryCount))
            return rawInput;

        const bool explicitInput = lutInputValues.size() == lutOutputValues.size();
        const auto inMin = inputMinEditor.getText().trim().getDoubleValue();
        const auto inMax = inputMaxEditor.getText().trim().getDoubleValue();
        if (!explicitInput && inMax <= inMin)
            return rawInput;

        const auto normalized = explicitInput
            ? 0.0
            : juce::jlimit(0.0, 1.0, (static_cast<double>(rawInput) - inMin) / (inMax - inMin));

#if LININTERPOL_HAVE_ALGLIB
        if (splineReady)
        {
            if (splineUsesExplicitInputAxis)
            {
                const auto x = juce::jlimit(splineXMin, splineXMax, static_cast<double>(rawInput));
                return static_cast<float>(alglib::spline1dcalc(lutSpline, x));
            }
            return static_cast<float>(alglib::spline1dcalc(lutSpline, normalized));
        }
#endif

        if (explicitInput)
        {
            if (rawInput <= lutInputValues.front())
                return lutOutputValues.front();
            if (rawInput >= lutInputValues.back())
                return lutOutputValues.back();

            const auto upper = std::upper_bound(lutInputValues.begin(), lutInputValues.end(), rawInput);
            const auto upperIdx = static_cast<size_t>(std::distance(lutInputValues.begin(), upper));
            const auto lowerIdx = upperIdx - 1;

            const auto x0 = static_cast<double>(lutInputValues[lowerIdx]);
            const auto x1 = static_cast<double>(lutInputValues[upperIdx]);
            const auto y0 = lutOutputValues[lowerIdx];
            const auto y1 = lutOutputValues[upperIdx];
            const auto frac = static_cast<float>((static_cast<double>(rawInput) - x0) / (x1 - x0));
            return y0 + frac * (y1 - y0);
        }

        const auto lutIndex = normalized * static_cast<double>(lutOutputValues.size() - 1);

        const auto lowerIndex = static_cast<int>(std::floor(lutIndex));
        const auto upperIndex = juce::jmin(lowerIndex + 1, static_cast<int>(lutOutputValues.size() - 1));
        const auto frac = static_cast<float>(lutIndex - static_cast<double>(lowerIndex));

        return lutOutputValues[static_cast<size_t>(lowerIndex)]
            + frac * (lutOutputValues[static_cast<size_t>(upperIndex)] - lutOutputValues[static_cast<size_t>(lowerIndex)]);
    }

    float getNearestLutSample(float rawInput) const
    {
        if (lutOutputValues.empty())
            return rawInput;

        if (lutInputValues.size() == lutOutputValues.size())
        {
            if (rawInput <= lutInputValues.front())
                return lutOutputValues.front();
            if (rawInput >= lutInputValues.back())
                return lutOutputValues.back();

            const auto upper = std::upper_bound(lutInputValues.begin(), lutInputValues.end(), rawInput);
            const auto upperIdx = static_cast<size_t>(std::distance(lutInputValues.begin(), upper));
            const auto lowerIdx = upperIdx - 1;
            const auto x0 = static_cast<double>(lutInputValues[lowerIdx]);
            const auto x1 = static_cast<double>(lutInputValues[upperIdx]);
            if (std::abs(x1 - x0) < 1.0e-12)
                return lutOutputValues[lowerIdx];
            const auto y0 = lutOutputValues[lowerIdx];
            const auto y1 = lutOutputValues[upperIdx];
            const auto frac = static_cast<float>((static_cast<double>(rawInput) - x0) / (x1 - x0));
            return y0 + frac * (y1 - y0);
        }

        const auto inMin = inputMinEditor.getText().trim().getDoubleValue();
        const auto inMax = inputMaxEditor.getText().trim().getDoubleValue();
        if (inMax <= inMin)
            return rawInput;

        const auto normalized = juce::jlimit(0.0, 1.0, (static_cast<double>(rawInput) - inMin) / (inMax - inMin));
        const auto lutIndex = normalized * static_cast<double>(lutOutputValues.size() - 1);
        const auto lowerIndex = static_cast<int>(std::floor(lutIndex));
        const auto upperIndex = juce::jmin(lowerIndex + 1, static_cast<int>(lutOutputValues.size() - 1));
        const auto frac = static_cast<float>(lutIndex - static_cast<double>(lowerIndex));

        return lutOutputValues[static_cast<size_t>(lowerIndex)]
            + frac * (lutOutputValues[static_cast<size_t>(upperIndex)] - lutOutputValues[static_cast<size_t>(lowerIndex)]);
    }

    void closeFreeDConnection()
    {
        freeDSocket.reset();
        freeDConnected = false;
    }

    void applyFreeDConnection()
    {
        closeFreeDConnection();
        freeDPacketsReceived = 0;
        freeDLastCameraId = 0;
        freeDLastZoom = 0;
        freeDLastFocus = 0;

        if (!freeDEnableToggle.getToggleState())
            return;

        const auto port = freeDPortEditor.getText().trim().getIntValue();
        if (port <= 0 || port > 65535)
            return;

        auto socket = std::make_unique<juce::DatagramSocket>(false);
        socket->setEnablePortReuse(true);
        if (!socket->bindToPort(port))
            return;

        freeDSocket = std::move(socket);
        freeDConnected = true;
    }

    void restartFreeDListener()
    {
        applyFreeDConnection();
        updateConnectionStatusLabel();
        if (!freeDEnableToggle.getToggleState())
        {
            freeDStatusLabel.setText("FreeD: disabled", juce::dontSendNotification);
        }
        else if (freeDConnected)
        {
            freeDStatusLabel.setText(
                "FreeD: listener restarted on " + freeDPortEditor.getText().trim(),
                juce::dontSendNotification);
        }
        else
        {
            freeDStatusLabel.setText(
                "FreeD: restart failed on " + freeDPortEditor.getText().trim(),
                juce::dontSendNotification);
        }
    }

    void pollFreeDInput()
    {
        if (!freeDConnected || freeDSocket == nullptr)
            return;

        std::array<std::uint8_t, 512> buffer {};
        constexpr int kMaxPacketsPerTick = 64;
        for (int i = 0; i < kMaxPacketsPerTick; ++i)
        {
            if (freeDSocket->waitUntilReady(true, 0) <= 0)
                break;

            const auto bytesRead = freeDSocket->read(buffer.data(), static_cast<int>(buffer.size()), false);
            if (bytesRead <= 0)
                break;

            ParsedFreeD1Packet packet;
            if (!parseFreeD1Packet(buffer.data(), bytesRead, packet))
                continue;

            rawValue.store(static_cast<float>(packet.zoom), std::memory_order_relaxed);
            focusValue.store(static_cast<float>(packet.focus), std::memory_order_relaxed);
            receivedCount.fetch_add(1, std::memory_order_relaxed);
            ++freeDPacketsReceived;
            freeDLastCameraId = packet.cameraId;
            freeDLastZoom = packet.zoom;
            freeDLastFocus = packet.focus;
        }
    }

    void updateConnectionStatusLabel()
    {
        juce::String freeDState;
        if (!freeDEnableToggle.getToggleState())
        {
            freeDState = "disabled";
        }
        else if (freeDConnected)
        {
            freeDState = "listening:" + freeDPortEditor.getText().trim();
        }
        else
        {
            freeDState = "failed";
        }

        connectionStatusLabel.setText(
            "Input: " + juce::String(inputConnected ? "connected" : "failed")
                + " | Output: " + juce::String(outputConnected ? "connected" : "failed")
                + " | FreeD: " + freeDState,
            juce::dontSendNotification);
    }

    void applyConnections()
    {
        disconnect();
        inputConnected = connect(inPortEditor.getText().trim().getIntValue());

        sender.disconnect();
        outputConnected = sender.connect(outHostEditor.getText().trim(), outPortEditor.getText().trim().getIntValue());
        applyFreeDConnection();
        updateConnectionStatusLabel();
    }

    void restartOutputTimer()
    {
        const auto hz = juce::jlimit(1, 240, static_cast<int>(std::lround(outputRateSlider.getValue())));
        startTimerHz(hz);
    }

    juce::Label titleLabel;

    juce::Label inPortLabel;
    juce::Label outHostLabel;
    juce::Label outPortLabel;
    juce::Label freeDPortLabel;
    juce::Label inAddressLabel;
    juce::Label focusAddressLabel;
    juce::Label focusInMinLabel;
    juce::Label focusInMaxLabel;
    juce::Label outAddressLabel;

    juce::TextEditor inPortEditor;
    juce::TextEditor outHostEditor;
    juce::TextEditor outPortEditor;
    juce::TextEditor freeDPortEditor;
    juce::TextEditor inAddressEditor;
    juce::TextEditor focusAddressEditor;
    juce::TextEditor focusInMinEditor;
    juce::TextEditor focusInMaxEditor;
    juce::TextEditor outAddressEditor;
    juce::ToggleButton freeDEnableToggle;
    juce::TextButton restartFreeDButton;
    juce::TextButton applyButton;

    juce::Label inputMinLabel;
    juce::Label inputMaxLabel;
    juce::TextEditor inputMinEditor;
    juce::TextEditor inputMaxEditor;
    juce::TextButton loadLutButton;
    juce::TextButton saveLutButton;
    juce::TextButton loadSurfaceButton;
    juce::TextButton newLutButton;
    juce::TextButton addPointAtInputButton;
    juce::TextButton undoLutButton;
    juce::TextButton redoLutButton;
    juce::Label lutStatusLabel;
    juce::Label pointEditLabel;
    juce::TextEditor pointEditValueEditor;
    juce::TextButton pointDec1Button;
    juce::TextButton pointInc1Button;
    juce::TextButton pointDec01Button;
    juce::TextButton pointInc01Button;
    juce::TextButton pointDec001Button;
    juce::TextButton pointInc001Button;
    juce::Label graphZoomLabel;
    juce::Slider graphZoomSlider;
    juce::TextButton resetGraphZoomButton;

    juce::Label smoothingLabel;
    juce::Label outputRateLabel;
    juce::Slider smoothingSlider;
    juce::Slider outputRateSlider;

    juce::Label presetNameLabel;
    juce::TextEditor presetNameEditor;
    juce::TextButton loadPresetButton;
    juce::TextButton savePresetButton;
    juce::Label settingsStatusLabel;

    juce::Label rawValueLabel;
    juce::Label focusValueLabel;
    juce::Label freeDStatusLabel;
    juce::Label targetValueLabel;
    juce::Label interpValueLabel;
    juce::Label receivedCountLabel;
    juce::Label connectionStatusLabel;
    HistoryPlot historyPlot;
    LutCartesianPlot lutCartesianPlot;

    juce::OSCSender sender;
    std::unique_ptr<juce::DatagramSocket> freeDSocket;

    std::atomic<float> rawValue { 0.0f };
    std::atomic<float> focusValue { 0.0f };
    std::atomic<int> receivedCount { 0 };

    std::vector<float> lutInputValues;
    std::vector<float> lutOutputValues;
    std::unique_ptr<juce::FileChooser> lutFileChooser;
    std::unique_ptr<juce::FileChooser> saveLutFileChooser;
    std::unique_ptr<juce::FileChooser> newLutFileChooser;
    std::unique_ptr<juce::FileChooser> surfaceFileChooser;
    juce::String lastLutFilePath;
    juce::String lastSurfaceFilePath;
    std::vector<float> pendingNewLutInputValues;
    std::vector<float> pendingNewLutOutputValues;
    std::vector<float> pendingNewSurfaceFocusValues;
    bool pendingNewCreatesSurface = false;
    std::vector<float> surfaceLensAxis;
    std::vector<float> surfaceFocusAxis;
    std::vector<float> surfaceValuesGrid;
    bool newLutCaptureActive = false;
    bool newLutHasObservedRaw = false;
    bool newLutHasObservedFocus = false;
    float newLutObservedMin = 0.0f;
    float newLutObservedMax = 0.0f;
    float newLutObservedFocusMin = 0.0f;
    float newLutObservedFocusMax = 0.0f;
    std::vector<LutSnapshot> lutUndoStack;
    std::vector<LutSnapshot> lutRedoStack;
    LutSnapshot dragStartSnapshot;
    bool dragHistoryActive = false;
    int selectedLutPointIndex = -1;
    int selectedSurfaceLensIndex = -1;
    int selectedSurfaceFocusIndex = -1;
    bool suppressPointEditorCallbacks = false;
    std::vector<float> cubicCurveX;
    std::vector<float> cubicCurveY;
    double lastCurveInMin = 0.0;
    double lastCurveInMax = 0.0;
    bool cubicCurveDirty = true;

#if LININTERPOL_HAVE_ALGLIB
    alglib::spline1dinterpolant lutSpline;
    alglib::spline2dinterpolant surfaceSpline;
    bool splineReady = false;
    bool surfaceSplineReady = false;
    bool splineUsesExplicitInputAxis = false;
    double splineXMin = 0.0;
    double splineXMax = 1.0;
    double surfaceLensMin = 0.0;
    double surfaceLensMax = 1.0;
    double surfaceFocusMin = 0.0;
    double surfaceFocusMax = 1.0;
#endif

    float interpolatedValue = 0.0f;
    bool inputConnected = false;
    bool outputConnected = false;
    bool freeDConnected = false;
    int freeDLastCameraId = 0;
    int freeDLastZoom = 0;
    int freeDLastFocus = 0;
    size_t freeDPacketsReceived = 0;
    double lastTickSeconds = 0.0;
};

class MainWindow final : public juce::DocumentWindow
{
public:
    explicit MainWindow(juce::String name)
        : juce::DocumentWindow(std::move(name),
                               juce::Desktop::getInstance().getDefaultLookAndFeel().findColour(juce::ResizableWindow::backgroundColourId),
                               juce::DocumentWindow::allButtons)
    {
        setUsingNativeTitleBar(true);
        setResizable(true, true);
        setResizeLimits(680, 940, 1400, 1400);
        setContentOwned(new MainComponent(), true);
        centreWithSize(getWidth(), getHeight());
        setVisible(true);
    }

    void closeButtonPressed() override
    {
        juce::JUCEApplication::getInstance()->systemRequestedQuit();
    }
};

class LinInterpolOscGuiApplication final : public juce::JUCEApplication
{
public:
    LinInterpolOscGuiApplication() = default;

    const juce::String getApplicationName() override { return "LinInterpol OSC Starter"; }
    const juce::String getApplicationVersion() override { return "0.1.0"; }
    bool moreThanOneInstanceAllowed() override { return true; }

    void initialise(const juce::String&) override
    {
        mainWindow = std::make_unique<MainWindow>(getApplicationName());
    }

    void shutdown() override
    {
        mainWindow.reset();
    }

    void systemRequestedQuit() override
    {
        quit();
    }

private:
    std::unique_ptr<MainWindow> mainWindow;
};

START_JUCE_APPLICATION(LinInterpolOscGuiApplication)
