#pragma once
#include "ProtocolSession.h"
#include "ScopeData.h"
#include "ScopeRecording.h"
#include <QElapsedTimer>

// Lives with ProtocolSession in the I/O thread. No per-sample GUI signals.
class CaptureEngine : public QObject {
    Q_OBJECT
public:
    CaptureEngine(ProtocolSession *,std::shared_ptr<ScopeBuffer>,QObject *parent=nullptr);
    void install(const QByteArray &capabilities);
    void begin(const QVector<Entry> &,int sampleHz,int requestedMode,bool singleShot=false);
    void stop();
    void disconnected(const QString &);
    bool active() const;
    static int chooseMode(double payloadBps,double sampleHz,int stride,int blockBytes=1024);
    void setRecording(const QString &path,const QByteArray &hash) {m_recordPath=path;m_dictionaryHash=hash;}
signals:
    void metrics(QVariantMap values);
private:
    enum Phase { Offline,Idle,Benchmark,Configure,Starting,Capturing,Stopping,Draining,Complete,Error } m_phase=Offline;
    enum Operation { None,BenchAck,ConfigAck,StartAck,StatusAck,StopAck,FinalAck } m_operation=None;
    ProtocolSession *m_session;
    std::shared_ptr<ScopeBuffer> m_buffer;
    QVector<Entry> m_channels;
    QTimer *m_timer;
    QElapsedTimer m_activity,m_statusClock,m_benchClock,m_totalClock;
    QByteArray m_caps;
    quint32 m_hz=0,m_memory=0,m_config=0,m_segment=0,m_nextBlock=0,m_baselineOverflow=0;
    quint64 m_nextTick=0,m_received=0,m_total=0,m_expected=0;
    quint32 m_divisor=1,m_overflows=0,m_segments=0;
    int m_stride=0,m_blockBytes=1024,m_maxChannels=0,m_rate=0,m_requestedMode=0,m_mode=2;
    int m_token=-1000,m_pending=0,m_benchRound=0,m_benchSequence=0;
    double m_measured=0;
    bool m_wantStop=false,m_single=false,m_started=false,m_switchBurst=false,m_dirty=false;
    QString m_error,m_afterDrainError;
    ScopeRecorder m_recorder;
    QString m_recordPath;
    QByteArray m_dictionaryHash;
    bool m_recording=false,m_repeatBurst=false;
    void record(quint8,quint32,const QByteArray &);
    void completeRecording(const QString &error={});
    void request(Operation,quint8,const QByteArray &payload={});
    void result(int,int,const QByteArray &);
    void stream(int,quint32,const QByteArray &);
    void configure();
    void startSegment();
    void sendStop();
    void finishDrain();
    void fail(const QString &);
    void notify();
};
