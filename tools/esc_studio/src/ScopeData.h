#pragma once
#include "Dictionary.h"
#include <QMutex>
#include <QPointF>
#include <QVector>
#include <atomic>
#include <deque>
#include <memory>

struct ScopeBlock {
    quint32 config=0,segment=0,sequence=0,hz=0,divisor=0;
    quint64 tick=0;
    quint16 count=0,stride=0;
    QByteArray raw;
    QVector<double> values;
    struct Peak { double low=0,high=0; quint16 lowAt=0,highAt=0; bool finite=false; };
    QVector<Peak> peaks; // One aggregate per 16 samples per channel.
    double start() const { return double(tick)/hz; }
    double end() const { return (double(tick)+double(count-1)*divisor)/hz; }
    qsizetype memory() const;
    static std::shared_ptr<ScopeBlock> decode(const QByteArray &,quint32 sequence,const QVector<Entry> &,QString *error);
};
struct ScopeEnvelope {
    QVector<QVector<QPointF>> curves; // NaN separates segments/invalid values.
    double low=0,high=0;
    qsizetype sourceSamples=0,vertices=0;
};
class ScopeBuffer {
public:
    explicit ScopeBuffer(qsizetype capacity=32*1024*1024):m_capacity(capacity) {}
    void clear(int channels);
    bool append(std::shared_ptr<const ScopeBlock>);
    QVector<std::shared_ptr<const ScopeBlock>> snapshot() const;
    ScopeEnvelope envelope(double from,double to,int pixels) const;
    QVariantMap inspect(double time) const;
    QVariantMap summary() const;
    quint64 generation() const { return m_generation.load(std::memory_order_relaxed); }
private:
    mutable QMutex m_mutex;
    std::deque<std::shared_ptr<const ScopeBlock>> m_blocks;
    qsizetype m_capacity,m_used=0;
    quint64 m_received=0,m_evicted=0;
    int m_channels=0;
    std::atomic<quint64> m_generation{0};
};
