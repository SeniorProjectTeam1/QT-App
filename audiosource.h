// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef AUDIOINPUT_H
#define AUDIOINPUT_H

#include <QAudioSource>
#include <QAudioSink>
#include <QAudioFormat>
#include <QMediaDevices>
#include <QMediaPlayer>
#include <QAudioOutput>

#include <QComboBox>
#include <QPushButton>
#include <QSlider>
#include <QWidget>

#include <QByteArray>
#include <QBuffer>

#include <memory>

class AudioInfo : public QIODevice
{
    Q_OBJECT

public:
    AudioInfo(const QAudioFormat &format);

    void start();
    void stop();

    qreal level() const { return m_level; }

    qint64 readData(char *data, qint64 maxlen) override;
    qint64 writeData(const char *data, qint64 len) override;

    qreal calculateLevel(const char *data, qint64 len) const;

    const QByteArray &buffer() const { return m_buffer; }
    const QAudioFormat &format() const { return m_format; }

signals:
    void levelChanged(qreal level);

private:
    const QAudioFormat m_format;
    qreal m_level = 0.0;      // 0.0 <= m_level <= 1.0
    QByteArray m_buffer;      // recorded raw audio data
};

class RenderArea : public QWidget
{
    Q_OBJECT

public:
    explicit RenderArea(QWidget *parent = nullptr);

    void setLevel(qreal value);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    qreal m_level = 0;
};

class InputTest : public QWidget
{
    Q_OBJECT

public:
    InputTest();

private:
    void initializeWindow();
    void initializeAudio(const QAudioDevice &deviceInfo);
    void initializeErrorWindow();

private slots:
    void init();
    void toggleRecording();
    void togglePause();
    void playLast();
    void saveRecording();
    void openAndPlayFile();
    void deviceChanged(int index);
    void sliderChanged(int value);
    void updateAudioDevices();

private:
    // Owned by layout
    RenderArea   *m_canvas            = nullptr;
    QPushButton  *m_startStopButton   = nullptr;
    QPushButton  *m_pauseResumeButton = nullptr;
    QPushButton  *m_playButton        = nullptr;
    QPushButton  *m_openFileButton    = nullptr;
    QPushButton  *m_saveButton        = nullptr;
    QComboBox    *m_deviceBox         = nullptr;
    QSlider      *m_volumeSlider      = nullptr;

    QMediaDevices                  *m_devices     = nullptr;
    std::unique_ptr<AudioInfo>      m_audioInfo;
    std::unique_ptr<QAudioSource>   m_audioSource;
    std::unique_ptr<QAudioSink>     m_audioSink;      // for raw buffer playback
    std::unique_ptr<QBuffer>        m_playBuffer;

    // For file playback
    QMediaPlayer  *m_filePlayer      = nullptr;
    QAudioOutput  *m_fileAudioOutput = nullptr;

    bool m_recording = false;
};

#endif // AUDIOINPUT_H
