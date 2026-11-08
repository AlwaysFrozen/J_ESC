// Bench-only probe: reads the enable state; only scope/benchmark commands are sent.
#include "CaptureEngine.h"
#include "ScopeRecording.h"
#include "ParameterScheme.h"
#include <QCoreApplication>
#include <QSerialPort>
#include <QFile>
#include <QCryptographicHash>
#include <QJsonDocument>
#include <QDebug>
#include <cstdio>

int main(int argc,char **argv)
{
    QCoreApplication app(argc,argv);auto args=app.arguments();
    qInstallMessageHandler([](QtMsgType,const QMessageLogContext &,const QString &message){
        auto bytes=message.toUtf8();std::fwrite(bytes.constData(),1,size_t(bytes.size()),stderr);std::fputc('\n',stderr);std::fflush(stderr);
    });
    auto option=[&](const QString &key,const QString &fallback=QString()) {int i=args.indexOf(key);return i>=0 && i+1<args.size()?args[i+1]:fallback;};
    QString input=option("--input");
    if(!input.isEmpty()) {
        QVariantMap result;QString error;auto buffer=std::make_shared<ScopeBuffer>();
        bool ok=ScopeRecording::read(input,buffer,option("--csv"),&result,&error);
        result["error"]=error;result["valid"]=ok;qInfo().noquote()<<QJsonDocument::fromVariant(result).toJson(QJsonDocument::Compact);
        return ok?0:1;
    }
    bool durationOk=false;
    int duration=option("--milliseconds","3000").toInt(&durationOk);
    if(!durationOk || duration<1 || duration>3600000){qCritical()<<"--milliseconds must be between 1 and 3600000";return 2;}
    Dictionary dictionary;QString error;
    QString dictionaryPath=option("--dictionary","canopen/object_dictionary.csv");
    if(!dictionary.load(dictionaryPath,&error)){qCritical().noquote()<<error;return 1;}
    if(!option("--scheme-input").isEmpty()) {
        ParameterScheme scheme(&dictionary);
        bool ok=scheme.preview(QUrl::fromLocalFile(option("--scheme-input"))) && scheme.apply();
        if(ok && !option("--scheme-output").isEmpty())ok=scheme.save(QUrl::fromLocalFile(option("--scheme-output")));
        qInfo().noquote()<<QJsonDocument::fromVariant(QVariantMap{{"valid",ok},{"drafts",dictionary.dirtyCount()},{"message",scheme.message()}}).toJson(QJsonDocument::Compact);
        return ok?0:1;
    }
    QStringList names=option("--channels","motor_basic0_phase_current_U,motor_basic0_phase_current_V,motor_basic0_phase_current_W").split(',');
    QVector<Entry> channels;
    for(const auto &name:names){int row=dictionary.find(name);if(row<0)return 2;channels.append(*dictionary.entry(row));}
    int enabled=dictionary.find("motor_status0_is_enabled");if(enabled<0)return 2;
    QSerialPort port;ProtocolSession session;auto buffer=std::make_shared<ScopeBuffer>();CaptureEngine capture(&session,buffer);
    QFile dictionaryFile(dictionaryPath);dictionaryFile.open(QIODevice::ReadOnly);
    capture.setRecording(option("--output"),QCryptographicHash::hash(dictionaryFile.readAll(),QCryptographicHash::Sha256).toHex());
    QVariantMap last;bool started=false,stopping=false;
    QObject::connect(&session,&ProtocolSession::outgoing,&app,[&](const QByteArray &packet){if(port.write(packet)!=packet.size())session.abort("short write");});
    QObject::connect(&port,&QSerialPort::readyRead,&app,[&]{session.ingest(port.readAll());});
    QObject::connect(&session,&ProtocolSession::failed,&app,[&](const QString &why){capture.disconnected(why);qCritical().noquote()<<why;app.exit(1);});
    QObject::connect(&session,&ProtocolSession::ready,&app,[&](const QByteArray &caps){
        capture.install(caps);session.enqueue(1,OD_MSG_READ,dictionary.entry(enabled)->requestHeader());
    });
    QObject::connect(&session,&ProtocolSession::result,&app,[&](int token,int code,const QByteArray &data){
        if(token!=1)return;
        if(code || dictionary.entry(enabled)->decode(data)!="0"){qCritical()<<"Device must be disabled";app.exit(3);return;}
        started=true;capture.begin(channels,option("--rate","20000").toInt(),option("--mode","0").toInt());
        QTimer::singleShot(duration,&app,[&]{stopping=true;capture.stop();});
    });
    QObject::connect(&capture,&CaptureEngine::metrics,&app,[&](const QVariantMap &metrics){
        last=metrics;
        if(started && stopping && !capture.active() && !metrics.value("recordBusy").toBool()) {
            qInfo().noquote()<<QJsonDocument::fromVariant(metrics).toJson(QJsonDocument::Compact);
            app.exit(metrics.value("error").toString().isEmpty() && metrics.value("recordError").toString().isEmpty()?0:1);
        }
    });
    port.setPortName(option("--port","COM3"));port.setBaudRate(115200);
    if(!port.open(QIODevice::ReadWrite)){qCritical()<<port.errorString();return 1;}
    port.setDataTerminalReady(true);port.setRequestToSend(true);port.clear();session.start();
    QTimer::singleShot(duration+20000,&app,[&]{qCritical()<<"Probe deadline"<<last;session.abort("probe deadline");app.exit(4);});
    int code=app.exec();port.setDataTerminalReady(false);port.close();return code;
}
