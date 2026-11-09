#pragma once
#include "Dictionary.h"
#include "ScopeViewModel.h"
#include "ParameterScheme.h"
#include <QObject>
#include <QHash>
#include <QThread>
#include <QTimer>
#include <QVariantList>

class Backend : public QObject {
    Q_OBJECT
    Q_PROPERTY(Dictionary* dictionary READ dictionary CONSTANT)
    Q_PROPERTY(ParameterFilter* parameters READ parameters CONSTANT)
    Q_PROPERTY(ScopeViewModel* scope READ scope CONSTANT)
    Q_PROPERTY(ParameterScheme* scheme READ scheme CONSTANT)
    Q_PROPERTY(QVariantMap motion READ motion NOTIFY statusChanged)
    Q_PROPERTY(QStringList ports READ ports NOTIFY portsChanged)
    Q_PROPERTY(bool connected READ connected NOTIFY stateChanged)
    Q_PROPERTY(bool connecting READ connecting NOTIFY stateChanged)
    Q_PROPERTY(bool busy READ busy NOTIFY stateChanged)
    Q_PROPERTY(bool controlBusy READ controlBusy NOTIFY stateChanged)
    Q_PROPERTY(int pendingCount READ pendingCount NOTIFY stateChanged)
    Q_PROPERTY(QString connectionLabel READ connectionLabel NOTIFY stateChanged)
    Q_PROPERTY(QString message READ message NOTIFY messageChanged)
    Q_PROPERTY(QStringList eventLog READ eventLog NOTIFY messageChanged)
    Q_PROPERTY(QVariantList motorStatus READ motorStatus NOTIFY statusChanged)
    Q_PROPERTY(QString configState READ configState NOTIFY statusChanged)
public:
    explicit Backend(QObject *parent=nullptr);
    ~Backend() override;
    Dictionary *dictionary() { return &m_dictionary; }
    ParameterFilter *parameters() { return &m_parameters; }
    ScopeViewModel *scope() { return m_scope; }
    ParameterScheme *scheme() {return &m_scheme;}
    QVariantMap motion() const {return m_motion;}
    Q_INVOKABLE void submitMotion(int mode,const QString &target,const QString &speed,const QString &acc,const QString &jerk);
    Q_INVOKABLE void stopMotion();
    Q_INVOKABLE void readAll();
    Q_INVOKABLE void writeAllChanges();
    Q_INVOKABLE bool saveScheme(const QUrl &u){return !busy() && m_scheme.save(u);}
    Q_INVOKABLE bool previewScheme(const QUrl &u){return !busy() && m_scheme.preview(u);}
    Q_INVOKABLE bool applyScheme(){return !busy() && m_scheme.apply();}
    QStringList ports() const { return m_ports; }
    bool connected() const { return m_connected; }
    bool connecting() const { return m_connecting; }
    bool busy() const {
        return m_scope->active() || controlBusy();
    }
    bool controlBusy() const {
        if(m_readOnConnect)return true;
        for(const auto &request:m_pending) if(request.command!="@status") return true;
        return false;
    }
    int pendingCount() const { return int(m_pending.size()); }
    QString connectionLabel() const { return m_label; }
    QString message() const { return m_message; }
    QStringList eventLog() const { return m_log; }
    QVariantList motorStatus() const;
    QString configState() const;
    Q_INVOKABLE void refreshPorts();
    Q_INVOKABLE void connectPort(const QString &);
    Q_INVOKABLE void disconnectPort();
    Q_INVOKABLE void openDemo();
    Q_INVOKABLE bool loadDictionary(const QUrl &);
    Q_INVOKABLE void readGroup(int);
    Q_INVOKABLE void writeGroup(int);
    Q_INVOKABLE void readRow(int);
    Q_INVOKABLE void refreshStatus();
    Q_INVOKABLE bool commandAvailable(const QString &) const;
    Q_INVOKABLE void command(const QString &,int identify=1);
signals:
    void portsChanged();
    void stateChanged();
    void statusChanged();
    void messageChanged();
private:
    struct Pending { int row; bool write; QByteArray bytes; QString command; };
    Dictionary m_dictionary;
    ParameterScheme m_scheme{&m_dictionary};
    ParameterFilter m_parameters;
    std::shared_ptr<ScopeBuffer> m_buffer;
    ScopeViewModel *m_scope;
    QThread m_thread;
    QObject *m_worker;
    QTimer m_poll;
    QHash<int,Pending> m_pending;
    int m_nextToken=0;
    bool m_connected=false,m_connecting=false;
    bool m_readOnConnect=false;
    QString m_batchLabel;
    int m_batchFailures=0;
    QString m_label=QStringLiteral("未连接"),m_message;
    QStringList m_ports,m_log;
    QVariantMap m_motion;
    void log(const QString &);
    void queue(int row,bool write=false,const QByteArray &value={},const QString &command={},bool priority=false);
    void receive(int token,int status,const QByteArray &);
    void startConnection(const QString &,bool demo);
};
