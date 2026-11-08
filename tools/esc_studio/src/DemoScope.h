#pragma once
#include "Dictionary.h"
#include "od_wire.h"
#include <QElapsedTimer>
#include <QQueue>
#include <QTimer>

// Explicit local demonstration transport, never a source of hardware benchmarks.
class DemoScope:public QObject {
    Q_OBJECT
public:
    explicit DemoScope(QObject *parent=nullptr);
    void reset();void shutdown();
    bool handle(const OD_WireFrame &,OD_WireFrame *,const QHash<quint32,Entry> &);
signals:
    void packet(QByteArray);
private:
    QTimer *m_timer;
    QElapsedTimer m_clock;
    QVector<Entry> m_channels;
    QQueue<QByteArray> m_ready;
    QByteArray m_partial;
    quint32 m_config=0,m_segment=0,m_sequence=0,m_captured=0,m_overflows=0,m_divisor=1;
    quint64 m_firstTick=0,m_partialTick=0;
    int m_stride=0,m_perBlock=0,m_partialCount=0,m_mode=2,m_state=0,m_benchmark=0,m_benchSequence=0;
    void produce();void publish();void status(OD_WireFrame *);
};
