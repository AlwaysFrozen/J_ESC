#pragma once
#include "od_wire.h"
#include <QByteArray>
#include <QObject>
#include <QQueue>
#include <QTimer>

class ProtocolSession : public QObject {
    Q_OBJECT
public:
    explicit ProtocolSession(QObject *parent = nullptr);
    void start(int handshakeTimeoutMs = 1500);
    void ingest(const QByteArray &);
    void enqueue(int token, quint8 kind, const QByteArray &, int timeoutMs = 1500, bool priority = false);
    void abort(const QString &reason);
    bool isReady() const { return m_state == Ready; }
    quint32 sessionId() const { return m_session; }
signals:
    void outgoing(QByteArray packet);
    void ready(QByteArray capabilities);
    void result(int token, int status, QByteArray data);
    void failed(QString reason);
    void stream(int kind, quint32 sequence, QByteArray payload);
private:
    enum State { Closed, Negotiating, Capabilities, Ready } m_state = Closed;
    struct Request { int token; quint8 kind; QByteArray payload; int timeoutMs; };
    QTimer *m_timeout;
    QByteArray m_text;
    OD_WireParser m_parser{};
    quint32 m_session = 0, m_sequence = 0;
    QQueue<Request> m_queue;
    Request m_current{};
    bool m_active = false;
    void send(quint8 kind, const QByteArray &);
    void receive(const OD_WireFrame &);
    void pump();
};
