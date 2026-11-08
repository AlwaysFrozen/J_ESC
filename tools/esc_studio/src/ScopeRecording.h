#pragma once
#include "ScopeData.h"
#include <QJsonObject>
#include <QMutex>
#include <QWaitCondition>
#include <QThread>
#include <deque>

// A bounded producer queue; all disk I/O belongs to the writer thread.
class ScopeRecorder : public QThread {
public:
    explicit ScopeRecorder(qsizetype budget=8*1024*1024):m_budget(budget) {}
    ~ScopeRecorder() override;
    bool begin(const QString &path,const QJsonObject &metadata,QString *error);
    bool append(quint8 type,quint32 session,quint32 sequence,const QByteArray &payload);
    void finish(const QString &error={});
    QVariantMap status() const;
protected:
    void run() override;
private:
    mutable QMutex m_mutex;
    QWaitCondition m_ready;
    std::deque<QByteArray> m_queue;
    qsizetype m_budget,m_queued=0;
    quint64 m_written=0;
    bool m_closing=false,m_done=true;
    QString m_path,m_error;
    QByteArray m_header;
};

namespace ScopeRecording {
QJsonObject metadata(const QVector<Entry> &,const QByteArray &caps,quint32 session,int rate,const QByteArray &dictionaryHash);
// Streaming validation and export; playback retains only the bounded display history.
bool read(const QString &path,std::shared_ptr<ScopeBuffer> buffer,const QString &csvPath,
          QVariantMap *result,QString *error,double windowEnd=-1,double windowSpan=0.1);
}
