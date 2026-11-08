#pragma once
#include "ScopeData.h"
#include <QObject>
#include <QTimer>
#include <QThread>
#include <QUrl>

class ScopeViewModel:public QObject {
    Q_OBJECT
    Q_PROPERTY(QVariantList choices READ choices NOTIFY choicesChanged)
    Q_PROPERTY(QStringList selectedNames READ selectedNames NOTIFY choicesChanged)
    Q_PROPERTY(QString search READ search WRITE setSearch NOTIFY choicesChanged)
    Q_PROPERTY(QVariantList legend READ legend NOTIFY statusChanged)
    Q_PROPERTY(QVariantMap info READ info NOTIFY statusChanged)
    Q_PROPERTY(QString state READ state NOTIFY statusChanged)
    Q_PROPERTY(bool active READ active NOTIFY statusChanged)
    Q_PROPERTY(bool connected READ connected NOTIFY statusChanged)
    Q_PROPERTY(int rate READ rate WRITE setRate NOTIFY statusChanged)
    Q_PROPERTY(int mode READ mode WRITE setMode NOTIFY statusChanged)
    Q_PROPERTY(bool singleShot READ singleShot WRITE setSingleShot NOTIFY statusChanged)
    Q_PROPERTY(double span READ span NOTIFY viewChanged)
    Q_PROPERTY(double viewStart READ viewStart NOTIFY viewChanged)
    Q_PROPERTY(double viewEnd READ viewEnd NOTIFY viewChanged)
    Q_PROPERTY(bool following READ following WRITE setFollowing NOTIFY viewChanged)
    Q_PROPERTY(double cursorA READ cursorA NOTIFY cursorChanged)
    Q_PROPERTY(double cursorB READ cursorB NOTIFY cursorChanged)
    Q_PROPERTY(QString cursorText READ cursorText NOTIFY cursorChanged)
    Q_PROPERTY(bool recordEnabled READ recordEnabled WRITE setRecordEnabled NOTIFY statusChanged)
    Q_PROPERTY(QUrl recordUrl READ recordUrl WRITE setRecordUrl NOTIFY statusChanged)
    Q_PROPERTY(bool fileBusy READ fileBusy NOTIFY statusChanged)
    Q_PROPERTY(QString fileMessage READ fileMessage NOTIFY statusChanged)
    Q_PROPERTY(QVariantMap analysis READ analysis NOTIFY analysisChanged)
    Q_PROPERTY(bool analysisBusy READ analysisBusy NOTIFY analysisChanged)
public:
    ScopeViewModel(std::shared_ptr<ScopeBuffer>,QObject *parent=nullptr);
    ~ScopeViewModel() override;
    void setDictionary(const Dictionary &);
    void setConnected(bool value);
    void acceptMetrics(const QVariantMap &);
    void reject(const QString &);
    std::shared_ptr<ScopeBuffer> buffer() const { return m_buffer; }
    QVariantList choices() const;
    QStringList selectedNames() const {return m_selected;}
    QString search() const {return m_search;}
    void setSearch(const QString &);
    QVariantList legend() const;
    QVariantMap info() const {return m_info;}
    QString state() const {return m_info.value("state",QStringLiteral("未连接")).toString();}
    bool active() const {return m_connected && (m_starting || m_info.value("active").toBool());}
    bool connected() const {return m_connected;}
    int rate() const {return m_rate;} void setRate(int);
    int mode() const {return m_mode;} void setMode(int);
    bool singleShot() const {return m_single;} void setSingleShot(bool);
    double span() const {return m_span;}
    double viewStart() const {return m_end-m_span;}
    double viewEnd() const {return m_end;}
    bool following() const {return m_follow;} void setFollowing(bool);
    double cursorA() const;double cursorB() const;
    QString cursorText() const;
    bool recordEnabled() const {return m_recordEnabled;}
    void setRecordEnabled(bool v){if(!active() && !fileBusy()){m_recordEnabled=v;emit statusChanged();}}
    QUrl recordUrl() const {return m_recordUrl;}
    void setRecordUrl(const QUrl &v){if(!active() && !fileBusy()){m_recordUrl=v;emit statusChanged();}}
    bool fileBusy() const {return m_fileBusy || m_info.value("recordBusy").toBool();}
    QString fileMessage() const {return m_fileMessage;}
    QVariantMap analysis() const {return m_analysis;}
    bool analysisBusy() const {return m_analysisBusy;}
    Q_INVOKABLE void analyze(int channel);
    Q_INVOKABLE void openRecording(const QUrl &);
    Q_INVOKABLE void exportRecording(const QUrl &source,const QUrl &destination);
    Q_INVOKABLE void seekRecording(double time);
    Q_INVOKABLE void selectChannel(const QString &,bool selected);
    Q_INVOKABLE void start();
    Q_INVOKABLE void stop();
    Q_INVOKABLE void zoom(double factor,double anchor=0.5);
    Q_INVOKABLE void pan(double fraction);
    Q_INVOKABLE void resetView();
    Q_INVOKABLE void setCursor(int which,double fraction);
signals:
    void choicesChanged();void statusChanged();void viewChanged();void cursorChanged();
    void startRequested(QVector<Entry> channels,int rate,int mode,bool singleShot);
    void stopRequested();
    void recordRequested(QString path);
    void analysisChanged();
private:
    std::shared_ptr<ScopeBuffer> m_buffer;
    QVector<Entry> m_choices;
    QStringList m_selected,m_displayNames;
    QString m_search;
    QVariantMap m_info;
    QTimer m_refresh;
    int m_rate=20000,m_mode=0;
    int m_refreshTick=0;
    bool m_connected=false,m_starting=false,m_single=false,m_follow=true,m_hasA=false,m_hasB=false;
    double m_span=0.1,m_end=0.1,m_a=0,m_b=0;
    void clampEnd();
    QThread m_fileThread;
    QObject *m_fileWorker;
    bool m_recordEnabled=false,m_fileBusy=false,m_replay=false;
    QUrl m_recordUrl;
    QString m_fileMessage;
    QVariantMap m_analysis;
    bool m_analysisBusy=false;
    void fileJob(const QUrl &,const QUrl &,bool playback,double windowEnd=-1);
};
