// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "audiosource.h"

#include <QAudioDevice>
#include <QAudioSource>
#include <QAudioSink>
#include <QAudio>
#include <QDateTime>
#include <QDebug>
#include <QLabel>
#include <QPainter>
#include <QVBoxLayout>
#include <QtEndian>

#include <QFile>
#include <QDataStream>
#include <QDir>
#include <QStandardPaths>
#include <QFileDialog>
#include <QMessageBox>
#include <QProcess>
#include <QUuid>
#include <QUrl>

#if QT_CONFIG(permissions)
#include <QCoreApplication>
#include <QPermission>
#endif

#include <cmath>
#include <vector>

// ===================== Helpers =====================

// --- WAV header helper ---
static void writeWavHeader(QIODevice &device,
                           const QAudioFormat &format,
                           qint64 dataSize)
{
    QDataStream out(&device);
    out.setByteOrder(QDataStream::LittleEndian);

    // RIFF chunk descriptor
    out.writeRawData("RIFF", 4);
    out << quint32(36 + dataSize);
    out.writeRawData("WAVE", 4);

    // fmt sub-chunk
    out.writeRawData("fmt ", 4);
    out << quint32(16);              // PCM chunk size
    out << quint16(1);               // PCM format
    out << quint16(format.channelCount());
    out << quint32(format.sampleRate());

    quint32 byteRate = format.sampleRate()
                       * format.channelCount()
                       * format.bytesPerSample();
    out << byteRate;

    quint16 blockAlign = format.channelCount() * format.bytesPerSample();
    out << blockAlign;

    out << quint16(8 * format.bytesPerSample()); // bits per sample

    // data sub-chunk
    out.writeRawData("data", 4);
    out << quint32(dataSize);
}

// --- MIDI helpers ---

// variable-length quantity
static void writeVariableLength(QDataStream &out, quint32 value)
{
    quint8 buffer[4];
    int index = 0;
    buffer[index++] = value & 0x7F;
    while ((value >>= 7) > 0) {
        buffer[index++] = (value & 0x7F) | 0x80;
    }
    for (int i = index - 1; i >= 0; --i)
        out << buffer[i];
}

struct MidiEvent {
    double startSec;
    double endSec;
    int    note;
    int    velocity;
};

// simple freq -> MIDI
static int freqToMidi(double freq)
{
    if (freq <= 0.0)
        return 0;
    double n = 69.0 + 12.0 * std::log2(freq / 440.0);
    return int(std::round(n));
}

// Very simple monophonic analysis: autocorrelation-based pitch per frame,
// then group frames into note events.
static QVector<MidiEvent> analyzeMonophonic(const QAudioFormat &format,
                                            const QByteArray &data)
{
    QVector<MidiEvent> events;

    if (format.sampleFormat() != QAudioFormat::Int16 || format.channelCount() < 1)
        return events; // only support mono Int16 here

    const int channels   = format.channelCount();
    const int sampleRate = format.sampleRate();
    const int bytesPerFrame = format.bytesPerFrame();

    int totalFrames = data.size() / bytesPerFrame;
    if (totalFrames <= 0)
        return events;

    // Convert to mono double samples (use first channel)
    std::vector<double> mono(totalFrames);
    const char *raw = data.constData();
    for (int i = 0; i < totalFrames; ++i) {
        const qint16 *frame = reinterpret_cast<const qint16 *>(raw + i * bytesPerFrame);
        mono[i] = double(frame[0]); // first channel
    }

    // Global RMS for threshold
    double sumsq = 0.0;
    for (double v : mono)
        sumsq += v * v;
    double globalRms = mono.empty() ? 0.0 : std::sqrt(sumsq / mono.size());
    double energyThreshold = globalRms * 0.1; // 10% of global RMS

    // Framing parameters
    const int winSize = 2048;
    const int hopSize = 1024;
    if (totalFrames < winSize)
        return events;

    // pitch range: 80 Hz – 1000 Hz
    int minLag = sampleRate / 1000;   // max freq ~ 1000 Hz
    int maxLag = sampleRate / 80;     // min freq ~ 80 Hz
    if (minLag < 1) minLag = 1;
    if (maxLag >= winSize) maxLag = winSize - 1;

    struct FramePitch {
        double timeSec;
        double freq;
    };
    QVector<FramePitch> frames;

    for (int start = 0; start + winSize <= totalFrames; start += hopSize) {
        // RMS
        double rms = 0.0;
        for (int n = 0; n < winSize; ++n) {
            double v = mono[start + n];
            rms += v * v;
        }
        rms = std::sqrt(rms / winSize);

        double freq = 0.0;

        if (rms > energyThreshold) {
            // naive autocorrelation
            int bestLag = 0;
            double bestCorr = 0.0;

            for (int lag = minLag; lag <= maxLag; ++lag) {
                double corr = 0.0;
                int limit = winSize - lag;
                for (int n = 0; n < limit; ++n)
                    corr += mono[start + n] * mono[start + n + lag];

                if (corr > bestCorr) {
                    bestCorr = corr;
                    bestLag = lag;
                }
            }

            if (bestLag > 0 && bestCorr > 0.0)
                freq = double(sampleRate) / double(bestLag);
        }

        double centerTime = double(start + winSize / 2) / double(sampleRate);
        frames.push_back({centerTime, freq});
    }

    // Segment into notes
    const double minNoteDur   = 0.08;   // seconds
    const double pitchTolSemi = 0.5;    // semitone tolerance

    bool inNote       = false;
    double noteStart  = 0.0;
    double lastTime   = 0.0;
    double accFreq    = 0.0;
    int    accCount   = 0;

    auto commitNote = [&](double endTime) {
        if (!inNote || accCount == 0)
            return;
        double dur = endTime - noteStart;
        if (dur < minNoteDur)
            return;

        double meanFreq = accFreq / accCount;
        int midi = freqToMidi(meanFreq);
        if (midi <= 0)
            return;

        MidiEvent ev;
        ev.startSec = noteStart;
        ev.endSec   = endTime;
        ev.note     = midi;
        ev.velocity = 100;
        events.push_back(ev);
    };

    for (int i = 0; i < frames.size(); ++i) {
        const auto &fp = frames[i];
        lastTime = fp.timeSec;

        if (fp.freq <= 0.0) {
            // silence / no pitch
            if (inNote) {
                commitNote(fp.timeSec);
                inNote = false;
                accFreq = 0.0;
                accCount = 0;
            }
            continue;
        }

        int frameMidi = freqToMidi(fp.freq);

        if (!inNote) {
            // start new note
            inNote = true;
            noteStart = fp.timeSec;
            accFreq = fp.freq;
            accCount = 1;
        } else {
            int currentMidi = freqToMidi(accFreq / accCount);
            if (std::abs(frameMidi - currentMidi) <= pitchTolSemi) {
                // same note, accumulate
                accFreq += fp.freq;
                accCount++;
            } else {
                // pitch jump -> close previous note, start new
                commitNote(fp.timeSec);
                inNote = true;
                noteStart = fp.timeSec;
                accFreq = fp.freq;
                accCount = 1;
            }
        }
    }

    if (inNote)
        commitNote(lastTime);

    return events;
}

// Write a real monophonic MIDI file from audio buffer analysis
static bool writeMidiFile(const QString &fileName,
                          const QAudioFormat &format,
                          const QByteArray &data,
                          QString *errorString = nullptr)
{
    QVector<MidiEvent> events = analyzeMonophonic(format, data);
    if (events.isEmpty()) {
        if (errorString)
            *errorString = QObject::tr("No detectable notes in recording.");
        return false;
    }

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly)) {
        if (errorString)
            *errorString = file.errorString();
        return false;
    }

    QDataStream out(&file);
    out.setByteOrder(QDataStream::BigEndian);

    // --- MIDI header chunk ---
    out.writeRawData("MThd", 4);
    out << quint32(6);        // header length
    out << quint16(0);        // format 0 (single track)
    out << quint16(1);        // one track
    quint16 division = 480;   // ticks per quarter note
    out << division;

    // We'll use 120 BPM for quantization
    const double bpm         = 120.0;
    const double ticksPerSec = division * bpm / 60.0;

    // --- Build track data ---
    QByteArray trackData;
    QDataStream t(&trackData, QIODevice::WriteOnly);
    t.setByteOrder(QDataStream::BigEndian);

    // Set tempo to 120 BPM -> 500000 microseconds per quarter note
    writeVariableLength(t, 0);
    t << quint8(0xFF) << quint8(0x51) << quint8(0x03);
    t << quint8(0x07) << quint8(0xA1) << quint8(0x20); // 500000

    // Program change: channel 0, program 0 (Acoustic Grand Piano)
    writeVariableLength(t, 0);
    t << quint8(0xC0) << quint8(0x00);

    // Note events
    double currentTimeSec = 0.0;

    for (const MidiEvent &ev : events) {
        if (ev.endSec <= ev.startSec)
            continue;

        quint32 startTick = quint32(ev.startSec * ticksPerSec);
        quint32 endTick   = quint32(ev.endSec   * ticksPerSec);
        if (endTick <= startTick)
            endTick = startTick + division / 8;

        quint32 prevTick = quint32(currentTimeSec * ticksPerSec);

        // delta to note-on
        quint32 dtOn = startTick - prevTick;
        writeVariableLength(t, dtOn);
        t << quint8(0x90) << quint8(ev.note) << quint8(ev.velocity);

        // delta to note-off
        quint32 dtOff = endTick - startTick;
        writeVariableLength(t, dtOff);
        t << quint8(0x80) << quint8(ev.note) << quint8(0x00);

        currentTimeSec = ev.endSec;
    }

    // End of track
    writeVariableLength(t, 0);
    t << quint8(0xFF) << quint8(0x2F) << quint8(0x00);

    // --- Write track chunk ---
    out.writeRawData("MTrk", 4);
    out << quint32(trackData.size());
    out.writeRawData(trackData.constData(), trackData.size());

    return true;
}

// Simple helper: convert temp WAV to MP3 with ffmpeg
static bool convertWavToMp3(const QString &wavPath,
                            const QString &mp3Path,
                            QString *errorString = nullptr)
{
    QString program = "ffmpeg";

    QStringList args;
    args << "-y" << "-i" << wavPath << mp3Path;

    int rc = QProcess::execute(program, args);
    if (rc != 0) {
        if (errorString)
            *errorString = QObject::tr("ffmpeg failed with exit code %1").arg(rc);
        return false;
    }

    return true;
}

// ===================== AudioInfo =====================

AudioInfo::AudioInfo(const QAudioFormat &format) : m_format(format) { }

void AudioInfo::start()
{
    m_buffer.clear();             // start a fresh recording
    open(QIODevice::WriteOnly);
}

void AudioInfo::stop()
{
    close();
}

qint64 AudioInfo::readData(char * /* data */, qint64 /* maxlen */)
{
    return 0;
}

qreal AudioInfo::calculateLevel(const char *data, qint64 len) const
{
    const int channelBytes = m_format.bytesPerSample();
    const int sampleBytes  = m_format.bytesPerFrame();
    const int numSamples   = len / sampleBytes;

    float maxValue = 0;
    auto *ptr = reinterpret_cast<const unsigned char *>(data);

    for (int i = 0; i < numSamples; ++i) {
        for (int j = 0; j < m_format.channelCount(); ++j) {
            float value = m_format.normalizedSampleValue(ptr);

            maxValue = qMax(value, maxValue);
            ptr += channelBytes;
        }
    }
    return maxValue;
}

qint64 AudioInfo::writeData(const char *data, qint64 len)
{
    m_level = calculateLevel(data, len);
    m_buffer.append(data, len);   // store audio samples

    emit levelChanged(m_level);

    return len;
}

// ===================== RenderArea =====================

RenderArea::RenderArea(QWidget *parent) : QWidget(parent)
{
    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);

    setMinimumHeight(30);
    setMinimumWidth(200);
}

void RenderArea::paintEvent(QPaintEvent * /* event */)
{
    QPainter painter(this);

    painter.setPen(Qt::black);

    const QRect frame = painter.viewport() - QMargins(10, 10, 10, 10);
    painter.drawRect(frame);
    if (m_level == 0.0)
        return;

    float remappedLevel = QtAudio::convertVolume(m_level, QtAudio::LinearVolumeScale,
                                                 QtAudio::LogarithmicVolumeScale);

    const int pos = qRound(qreal(frame.width() - 1) * remappedLevel);
    painter.fillRect(frame.left() + 1, frame.top() + 1, pos, frame.height() - 1, Qt::red);
}

void RenderArea::setLevel(qreal value)
{
    m_level = value;
    update();
}

// ===================== InputTest =====================

InputTest::InputTest() : m_devices(new QMediaDevices(this))
{
    init();
}

void InputTest::initializeWindow()
{
    QVBoxLayout *layout = new QVBoxLayout(this);

    m_canvas = new RenderArea(this);
    layout->addWidget(m_canvas);

    m_deviceBox = new QComboBox(this);
    connect(m_deviceBox, &QComboBox::activated, this, &InputTest::deviceChanged);
    connect(m_devices, &QMediaDevices::audioInputsChanged, this, &InputTest::updateAudioDevices);
    updateAudioDevices();
    layout->addWidget(m_deviceBox);

    m_volumeSlider = new QSlider(Qt::Horizontal, this);
    m_volumeSlider->setRange(0, 100);
    m_volumeSlider->setValue(100);
    connect(m_volumeSlider, &QSlider::valueChanged, this, &InputTest::sliderChanged);
    layout->addWidget(m_volumeSlider);

    // Start / Stop recording button
    m_startStopButton = new QPushButton(tr("Start"), this);
    connect(m_startStopButton, &QPushButton::clicked, this, &InputTest::toggleRecording);
    layout->addWidget(m_startStopButton);

    // Pause / Resume button (affects recording only)
    m_pauseResumeButton = new QPushButton(tr("Pause"), this);
    connect(m_pauseResumeButton, &QPushButton::clicked, this, &InputTest::togglePause);
    m_pauseResumeButton->setEnabled(false);
    layout->addWidget(m_pauseResumeButton);

    // Play last recording (from RAM buffer)
    m_playButton = new QPushButton(tr("Play last recording"), this);
    connect(m_playButton, &QPushButton::clicked, this, &InputTest::playLast);
    layout->addWidget(m_playButton);

    // Open and play file from disk
    m_openFileButton = new QPushButton(tr("Open audio file..."), this);
    connect(m_openFileButton, &QPushButton::clicked, this, &InputTest::openAndPlayFile);
    layout->addWidget(m_openFileButton);

    // Save recording to WAV / MP3 / MIDI
    m_saveButton = new QPushButton(tr("Save recording"), this);
    connect(m_saveButton, &QPushButton::clicked, this, &InputTest::saveRecording);
    layout->addWidget(m_saveButton);

    // File playback: QMediaPlayer + QAudioOutput
    m_filePlayer = new QMediaPlayer(this);
    m_fileAudioOutput = new QAudioOutput(QMediaDevices::defaultAudioOutput(), this);
    m_filePlayer->setAudioOutput(m_fileAudioOutput);
}

void InputTest::initializeAudio(const QAudioDevice &deviceInfo)
{
    QAudioFormat format;
    format.setSampleRate(44100);
    format.setChannelCount(1);
    format.setSampleFormat(QAudioFormat::Int16);

    bool sampleRateSupported = format.sampleRate() < deviceInfo.maximumSampleRate()
                               && format.sampleRate() > deviceInfo.minimumSampleRate();
    bool channelCountSupported = format.channelCount() < deviceInfo.maximumChannelCount()
                                 && format.channelCount() > deviceInfo.minimumChannelCount();

    if (!sampleRateSupported)
        format.setSampleRate(deviceInfo.preferredFormat().sampleRate());

    if (!channelCountSupported)
        format.setChannelCount(deviceInfo.preferredFormat().channelCount());

    m_audioInfo.reset(new AudioInfo(format));
    connect(m_audioInfo.get(), &AudioInfo::levelChanged, m_canvas, &RenderArea::setLevel);

    m_audioSource.reset(new QAudioSource(deviceInfo, format));
    qreal initialVolume = QAudio::convertVolume(m_audioSource->volume(),
                                                QAudio::LinearVolumeScale,
                                                QAudio::LogarithmicVolumeScale);
    m_volumeSlider->setValue(qRound(initialVolume * 100));

    // Output for playback (default output device, same format) for raw buffer
    m_audioSink.reset(new QAudioSink(QMediaDevices::defaultAudioOutput(), format));

    m_recording = false;
    m_startStopButton->setText(tr("Start"));
    m_pauseResumeButton->setText(tr("Pause"));
    m_pauseResumeButton->setEnabled(false);

    // Update Pause / Resume text based on recording state
    connect(m_audioSource.get(), &QAudioSource::stateChanged, this,
            [this](QAudio::State state) {
                switch (state) {
                case QAudio::ActiveState:
                    m_pauseResumeButton->setText(tr("Pause"));
                    m_pauseResumeButton->setEnabled(true);
                    break;
                case QAudio::SuspendedState:
                    m_pauseResumeButton->setText(tr("Resume"));
                    m_pauseResumeButton->setEnabled(true);
                    break;
                default:
                    m_pauseResumeButton->setEnabled(false);
                    break;
                }
            });

    // When raw-buffer playback finishes, close the play buffer
    connect(m_audioSink.get(), &QAudioSink::stateChanged, this,
            [this](QAudio::State state) {
                if (state == QAudio::IdleState || state == QAudio::StoppedState) {
                    if (m_playBuffer && m_playBuffer->isOpen())
                        m_playBuffer->close();
                }
            });
}

void InputTest::initializeErrorWindow()
{
    QVBoxLayout *layout = new QVBoxLayout(this);
    QLabel *errorLabel = new QLabel(tr("Microphone permission is not granted!"));
    errorLabel->setWordWrap(true);
    errorLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(errorLabel);
}

void InputTest::init()
{
#if QT_CONFIG(permissions)
    QMicrophonePermission microphonePermission;
    switch (qApp->checkPermission(microphonePermission)) {
    case Qt::PermissionStatus::Undetermined:
        qApp->requestPermission(microphonePermission, this, &InputTest::init);
        return;
    case Qt::PermissionStatus::Denied:
        qWarning("Microphone permission is not granted!");
        initializeErrorWindow();
        return;
    case Qt::PermissionStatus::Granted:
        break;
    }
#endif
    initializeWindow();
    initializeAudio(QMediaDevices::defaultAudioInput());
}

void InputTest::toggleRecording()
{
    if (!m_audioSource || !m_audioInfo)
        return;

    if (!m_recording) {
        // Start recording
        m_audioInfo->start();
        m_audioSource->start(m_audioInfo.get());
        m_startStopButton->setText(tr("Stop"));
        m_recording = true;
    } else {
        // Stop recording
        m_audioSource->stop();
        m_audioInfo->stop();
        m_startStopButton->setText(tr("Start"));
        m_pauseResumeButton->setEnabled(false);
        m_recording = false;
    }
}

void InputTest::togglePause()
{
    if (!m_audioSource || !m_recording)
        return;

    switch (m_audioSource->state()) {
    case QAudio::ActiveState:
        m_audioSource->suspend();
        break;
    case QAudio::SuspendedState:
        m_audioSource->resume();
        break;
    default:
        break;
    }
}

void InputTest::playLast()
{
    if (!m_audioInfo)
        return;

    const QByteArray &data = m_audioInfo->buffer();
    if (data.isEmpty()) {
        qWarning() << "No audio data to play.";
        QMessageBox::information(this, tr("No recording"),
                                 tr("There is no recorded audio to play yet."));
        return;
    }

    // If currently recording, stop recording first
    if (m_recording) {
        m_audioSource->stop();
        m_audioInfo->stop();
        m_startStopButton->setText(tr("Start"));
        m_pauseResumeButton->setEnabled(false);
        m_recording = false;
    }

    // Stop any file playback
    if (m_filePlayer)
        m_filePlayer->stop();

    if (!m_audioSink)
        m_audioSink.reset(new QAudioSink(QMediaDevices::defaultAudioOutput(),
                                         m_audioInfo->format()));

    m_audioSink->stop();

    m_playBuffer.reset(new QBuffer());
    m_playBuffer->setData(data);
    m_playBuffer->open(QIODevice::ReadOnly);

    m_audioSink->start(m_playBuffer.get());
}

void InputTest::openAndPlayFile()
{
    // Pick starting directory
    QString dirPath = QStandardPaths::writableLocation(QStandardPaths::MusicLocation);
    if (dirPath.isEmpty())
        dirPath = QDir::currentPath();

    QString filter = tr("Audio files (*.wav *.mp3 *.aac *.flac *.ogg);;All files (*.*)");
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open audio file"),
                                                    dirPath,
                                                    filter);
    if (fileName.isEmpty())
        return; // user cancelled

    // Stop raw-buffer playback if active
    if (m_audioSink)
        m_audioSink->stop();
    if (m_playBuffer && m_playBuffer->isOpen())
        m_playBuffer->close();

    // If we’re recording, stop so mic + file don’t clash
    if (m_recording) {
        m_audioSource->stop();
        m_audioInfo->stop();
        m_startStopButton->setText(tr("Start"));
        m_pauseResumeButton->setEnabled(false);
        m_recording = false;
    }

    if (!m_filePlayer || !m_fileAudioOutput) {
        QMessageBox::warning(this, tr("Playback error"),
                             tr("Internal player was not initialized correctly."));
        return;
    }

    m_filePlayer->stop();
    m_filePlayer->setSource(QUrl::fromLocalFile(fileName));
    m_filePlayer->play();
}

void InputTest::saveRecording()
{
    if (!m_audioInfo)
        return;

    const QByteArray &data = m_audioInfo->buffer();
    if (data.isEmpty()) {
        qWarning() << "No audio data to save.";
        QMessageBox::information(this, tr("No recording"),
                                 tr("There is no recorded audio to save yet."));
        return;
    }

    // Default directory and suggested file
    QString dirPath = QStandardPaths::writableLocation(QStandardPaths::MusicLocation);
    if (dirPath.isEmpty())
        dirPath = QDir::currentPath();

    QString defaultName = dirPath + QDir::separator() + "recording.wav";

    // Let the user choose WAV / MP3 / MIDI
    QString filter = tr("WAV files (*.wav);;MP3 files (*.mp3);;MIDI files (*.mid)");
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save recording"),
                                                    defaultName,
                                                    filter);
    if (fileName.isEmpty())
        return; // user canceled

    // If they didn't type an extension, default to WAV
    if (!fileName.contains('.'))
        fileName += ".wav";

    const QAudioFormat &fmt = m_audioInfo->format();

    // --- WAV ---
    if (fileName.endsWith(".wav", Qt::CaseInsensitive)) {
        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly)) {
            qWarning() << "Failed to open file for writing:" << file.errorString();
            QMessageBox::warning(this, tr("Save error"),
                                 tr("Failed to open file for writing:\n%1")
                                     .arg(file.errorString()));
            return;
        }

        writeWavHeader(file, fmt, data.size());
        file.write(data);
        file.close();

        qDebug() << "Saved WAV recording to" << fileName;
        QMessageBox::information(this, tr("Saved"),
                                 tr("Recording saved as WAV:\n%1").arg(fileName));
    }
    // --- MP3 (via ffmpeg) ---
    else if (fileName.endsWith(".mp3", Qt::CaseInsensitive)) {
        // 1) Write temp WAV
        QString tmpWavPath = QDir::tempPath()
                             + QDir::separator()
                             + QUuid::createUuid().toString(QUuid::WithoutBraces)
                             + ".wav";
        {
            QFile tmpFile(tmpWavPath);
            if (!tmpFile.open(QIODevice::WriteOnly)) {
                qWarning() << "Failed to open temp WAV for writing:" << tmpFile.errorString();
                QMessageBox::warning(this, tr("Save error"),
                                     tr("Failed to create temporary WAV file:\n%1")
                                         .arg(tmpFile.errorString()));
                return;
            }
            writeWavHeader(tmpFile, fmt, data.size());
            tmpFile.write(data);
            tmpFile.close();
        }

        // 2) Convert WAV -> MP3 with ffmpeg
        QString err;
        if (!convertWavToMp3(tmpWavPath, fileName, &err)) {
            qWarning() << "Failed to convert to MP3:" << err;
            QMessageBox::warning(
                this,
                tr("MP3 conversion error"),
                tr("Failed to convert WAV to MP3.\n"
                   "Make sure ffmpeg is installed and available in your PATH.\n\nDetails: %1")
                    .arg(err));
            QFile::remove(tmpWavPath);
            return;
        }

        QFile::remove(tmpWavPath);
        qDebug() << "Saved MP3 recording to" << fileName;
        QMessageBox::information(this, tr("Saved"),
                                 tr("Recording saved as MP3:\n%1").arg(fileName));
    }
    // --- MIDI (monophonic transcription) ---
    else if (fileName.endsWith(".mid", Qt::CaseInsensitive)
             || fileName.endsWith(".midi", Qt::CaseInsensitive)) {
        QString err;
        if (!writeMidiFile(fileName, fmt, data, &err)) {
            qWarning() << "Failed to write MIDI:" << err;
            QMessageBox::warning(this, tr("Save error"),
                                 tr("Failed to save MIDI file:\n%1").arg(err));
            return;
        }

        qDebug() << "Saved MIDI file to" << fileName;
        QMessageBox::information(this, tr("Saved"),
                                 tr("Recording saved as monophonic MIDI:\n%1").arg(fileName));
    }
    else {
        QMessageBox::warning(
            this,
            tr("Unknown format"),
            tr("The selected file extension is not supported.\n"
               "Please use .wav, .mp3, or .mid.")
            );
    }
}

void InputTest::deviceChanged(int index)
{
    if (m_audioSource)
        m_audioSource->stop();
    if (m_audioInfo)
        m_audioInfo->stop();

    m_recording = false;

    initializeAudio(m_deviceBox->itemData(index).value<QAudioDevice>());
}

void InputTest::sliderChanged(int value)
{
    if (!m_audioSource)
        return;

    qreal linearVolume = QAudio::convertVolume(value / qreal(100),
                                               QAudio::LogarithmicVolumeScale,
                                               QAudio::LinearVolumeScale);

    m_audioSource->setVolume(linearVolume);
}

void InputTest::updateAudioDevices()
{
    m_deviceBox->clear();
    const QAudioDevice &defaultDeviceInfo = QMediaDevices::defaultAudioInput();
    m_deviceBox->addItem(defaultDeviceInfo.description(), QVariant::fromValue(defaultDeviceInfo));
    for (auto &deviceInfo : m_devices->audioInputs()) {
        if (deviceInfo != defaultDeviceInfo)
            m_deviceBox->addItem(deviceInfo.description(), QVariant::fromValue(deviceInfo));
    }
}

#include "moc_audiosource.cpp"
