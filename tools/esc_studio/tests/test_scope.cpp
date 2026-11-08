#include "ScopeData.h"
#include "CaptureEngine.h"
#include "ScopeRecording.h"
#include "ScopeAnalysis.h"
#include <QTemporaryDir>
#include <QFile>
#include <QJsonArray>
#include <QtTest>
#include <QLibrary>
#include <QJsonDocument>
#include <cmath>
#include <cstring>

namespace {
Entry channel(quint16 type=8,quint8 size=4,quint8 sub=1)
{ Entry e;e.name=QString("runtime%1").arg(sub);e.index=0x2100;e.subindex=sub;e.type=type;e.size=size;e.access="ro";return e; }
QByteArray blockBytes(quint64 tick,quint32 sequence,quint32 segment,int count,const QVector<Entry> &channels)
{
    int stride=0;for(const auto &e:channels)stride+=e.size;
    QByteArray payload(32+count*stride,'\0');auto p=reinterpret_cast<uint8_t *>(payload.data());
    od_wire_put32(p,1);od_wire_put32(p+4,segment);od_wire_put32(p+8,sequence);od_wire_put32(p+12,20000);
    od_wire_put32(p+16,quint32(tick));od_wire_put32(p+20,quint32(tick>>32));od_wire_put32(p+24,1);
    od_wire_put16(p+28,quint16(count));od_wire_put16(p+30,quint16(stride));return payload;
}
std::shared_ptr<ScopeBlock> block(quint64 tick,quint32 sequence,quint32 segment=1,int count=256,double spike=0)
{
    auto channels=QVector<Entry>{channel()};auto payload=blockBytes(tick,sequence,segment,count,channels);
    for(int i=0;i<count;++i) {
        float value=tick+quint64(i)==12345?float(spike):0;
        quint32 bits;std::memcpy(&bits,&value,4);od_wire_put32(reinterpret_cast<uint8_t *>(payload.data()+32+4*i),bits);
    }
    QString error;return ScopeBlock::decode(payload,sequence,channels,&error);
}
}
class ScopeTests:public QObject {
    Q_OBJECT
private slots:
    void spectrumUsesRawContinuousSamples()
    {
        QVector<std::shared_ptr<const ScopeBlock>> blocks;
        for(int j=0;j<4;++j){auto bytes=blockBytes(quint64(j)*256,quint32(j),1,256,{channel()});
            for(int i=0;i<256;++i){float value=float(3+2*std::sin(2*std::acos(-1.0)*625*(j*256+i)/20000));quint32 u;std::memcpy(&u,&value,4);od_wire_put32(reinterpret_cast<uint8_t *>(bytes.data()+32+i*4),u);}
            QString error;blocks.append(ScopeBlock::decode(bytes,quint32(j),{channel()},&error));}
        auto result=ScopeAnalysis::analyze(blocks,0,0,1);QVERIFY(result["error"].toString().isEmpty());
        QCOMPARE(result["count"].toInt(),1024);QCOMPARE(result["peakHz"].toDouble(),625.0);
        QCOMPARE(result["points"].toList().back().toPointF().x(),10000.0);
        QVERIFY(std::abs(result["peakAmplitude"].toDouble()-2)<0.005);QVERIFY(std::abs(result["mean"].toDouble()-3)<0.0001);
        QVERIFY(std::abs(result["rms"].toDouble()-std::sqrt(11.0))<0.0001);
        blocks.append(block(100000,0,2,16)); // Outside the analyzed window must not erase valid samples.
        QCOMPARE(ScopeAnalysis::analyze(blocks,0,0,1)["count"].toInt(),1024);
        QVERIFY(!ScopeAnalysis::analyze(blocks,0,5,6)["error"].toString().isEmpty());
        QVERIFY(!ScopeAnalysis::analyze(blocks,5,0,1)["error"].toString().isEmpty());
    }
    void recordingRoundTripAndRejectsIncomplete()
    {
        QTemporaryDir dir;QVERIFY(dir.isValid());
        QByteArray caps(18,'\0');od_wire_put32(reinterpret_cast<uint8_t *>(caps.data()),20000);
        auto meta=ScopeRecording::metadata({channel()},caps,42,20000,"local-hash");
        QString path=dir.filePath("roundtrip.jescscope"),error;
        ScopeRecorder writer;QVERIFY(writer.begin(path,meta,&error));
        auto data=blockBytes(100,0,1,2,{channel()});
        auto finalStatus=[&](quint32 segment,int count) {
            QByteArray status(29,'\0');auto p=reinterpret_cast<uint8_t *>(status.data()+1);
            p[1]=2;p[2]=1;od_wire_put32(p+4,1);od_wire_put32(p+8,segment);od_wire_put32(p+12,quint32(count));
            od_wire_put32(p+20,1);od_wire_put16(p+24,4);od_wire_put16(p+26,256);return status;
        };
        QVERIFY(writer.append(OD_MSG_SCOPE_STATUS|OD_MSG_REPLY,42,9,finalStatus(1,2)));
        QVERIFY(writer.append(OD_MSG_SAMPLES,42,0,data));
        QVERIFY(writer.append(OD_MSG_SAMPLES,42,0,blockBytes(1000,0,2,3,{channel()})));
        QVERIFY(writer.append(OD_MSG_SCOPE_STATUS|OD_MSG_REPLY,42,10,finalStatus(2,3)));
        writer.finish();QVERIFY(writer.wait(3000));QVERIFY(writer.status()["recordComplete"].toBool());
        auto buffer=std::make_shared<ScopeBuffer>();QVariantMap result;
        QString csv=dir.filePath("out.csv");
        QVERIFY2(ScopeRecording::read(path,buffer,csv,&result,&error),qPrintable(error));
        QCOMPARE(result["received"].toULongLong(),5ULL);QCOMPARE(result["segments"].toInt(),2);
        auto window=std::make_shared<ScopeBuffer>();
        QVERIFY(ScopeRecording::read(path,window,{},&result,&error,0.0051,0.0002));
        QCOMPARE(window->snapshot().size(),1);QCOMPARE(window->snapshot().front()->segment,1U);
        QVERIFY(!buffer->inspect(0.025)["valid"].toBool());
        QFile exported(csv);QVERIFY(exported.open(QIODevice::ReadOnly));QVERIFY(exported.readAll().contains("\n\n2,1000,"));
        QVERIFY(!ScopeRecording::read(path,{},csv,&result,&error));
        QVERIFY(!writer.begin(path,meta,&error));
        QFile source(path);QVERIFY(source.open(QIODevice::ReadOnly));auto bytes=source.readAll();source.close();
        auto reject=[&](QByteArray damaged) {
            QFile out(dir.filePath("broken.jescscope"));QVERIFY(out.open(QIODevice::WriteOnly));out.write(damaged);out.close();
            QVERIFY(!ScopeRecording::read(out.fileName(),{},dir.filePath("failed.csv"),&result,&error));
            QVERIFY(!QFile::exists(dir.filePath("failed.csv")));
        };
        reject(bytes.left(bytes.size()-1));
        auto corrupt=bytes;corrupt[corrupt.size()-5]^=0x55;reject(corrupt);
        // Missing final status is incomplete even though all sample CRCs are valid.
        int start=9+4+QJsonDocument(meta).toJson(QJsonDocument::Compact).size();
        quint32 firstLength=od_wire_u32(reinterpret_cast<const uint8_t *>(bytes.constData()+start));
        reject(bytes.left(start)+bytes.mid(start+4+firstLength));
        ScopeRecorder tiny(1);QString tinyPath=dir.filePath("overflow.jescscope");QVERIFY(tiny.begin(tinyPath,meta,&error));
        QVERIFY(!tiny.append(OD_MSG_SAMPLES,42,0,data));QVERIFY(tiny.wait(3000));
        QVERIFY(!tiny.status()["recordError"].toString().isEmpty());QVERIFY(!tiny.status()["recordComplete"].toBool());
        QVERIFY(!QFile::exists(tinyPath));QVERIFY(QFile::exists(tinyPath+".partial"));
        QVERIFY(!ScopeRecording::read(tinyPath+".partial",{}, {},&result,&error));
        ScopeRecorder unwritable;QVERIFY(unwritable.begin(dir.filePath("missing/fail.jescscope"),meta,&error));
        QVERIFY(unwritable.wait(3000));QVERIFY(!unwritable.status()["recordError"].toString().isEmpty());
        // Display eviction must never reduce exported sample counts.
        auto small=std::make_shared<ScopeBuffer>(buffer->snapshot().back()->memory()+1);
        QVERIFY(ScopeRecording::read(path,small,{},&result,&error));
        QCOMPARE(result["received"].toULongLong(),5ULL);
        QVERIFY(small->summary()["evicted"].toULongLong()>0);
    }
    void validatesMixedScalarsAndLayout()
    {
        QVector<Entry> channels={channel(),channel(7,4,2),channel(2,1,3),channel(1,1,4)};
        QByteArray data=blockBytes(0x100000001ULL,0,1,2,channels);
        auto p=reinterpret_cast<uint8_t *>(data.data()+32);
        od_wire_put32(p,0x41480000);od_wire_put32(p+4,0xFFFFFFFF);p[8]=0xFF;p[9]=1;
        QString error;auto b=ScopeBlock::decode(data,0,channels,&error);QVERIFY2(b,qPrintable(error));
        QCOMPARE(b->tick,0x100000001ULL);QCOMPARE(b->values[0],12.5);QCOMPARE(b->values[1],4294967295.0);
        QCOMPARE(b->values[2],-1.0);QCOMPARE(b->values[3],1.0);
        QVERIFY(!ScopeBlock::decode(data,1,channels,&error));
        p[9]=2;QVERIFY(!ScopeBlock::decode(data,0,channels,&error));
        QVERIFY(!ScopeBlock::decode(data.left(data.size()-1),0,channels,&error));
    }
    void boundedHistoryRetainsDeviceTime()
    {
        auto b=block(0,0,1,64);QVERIFY(b);
        ScopeBuffer buffer(b->memory()*2+1);buffer.clear(1);
        for(int i=0;i<5;++i)QVERIFY(buffer.append(block(quint64(i)*64,quint32(i),1,64)));
        auto summary=buffer.summary();QCOMPARE(summary["received"].toULongLong(),320ULL);
        QCOMPARE(summary["evicted"].toULongLong(),3ULL);
        QVERIFY(summary["memory"].toLongLong()<=summary["capacity"].toLongLong());
        QVERIFY(!buffer.inspect(0).value("valid").toBool());
        QCOMPARE(buffer.inspect(200.0/20000)["tick"].toString(),QString("200"));
    }
    void envelopesPreserveSpikeOrderAndGaps()
    {
        ScopeBuffer buffer;buffer.clear(1);
        for(int i=0;i<100;++i)QVERIFY(buffer.append(block(quint64(i)*256,quint32(i),1,256,100)));
        QVERIFY(buffer.append(block(40000,0,2)));
        auto envelope=buffer.envelope(0,2.02,32);
        bool spike=false,gap=false,after=false;double previous=-1;
        for(const auto &point:envelope.curves[0]) {
            if(!std::isfinite(point.x())){gap=true;previous=-1;continue;}
            QVERIFY(point.x()>=previous);previous=point.x();
            if(point.y()==100){spike=true;QVERIFY(std::abs(point.x()-12345.0/20000)<1e-9);}
            if(point.x()>=2.0){after=true;QVERIFY(gap);}
        }
        QVERIFY(spike && after);QVERIFY(envelope.vertices<150);
        QVERIFY(!buffer.inspect(1.7)["valid"].toBool());
    }
    void modeBudgetIsConservative()
    {
        QCOMPARE(CaptureEngine::chooseMode(700000,20000,16),1);
        QCOMPARE(CaptureEngine::chooseMode(700000,20000,32),2);
        QCOMPARE(CaptureEngine::chooseMode(0,20000,4),2);
        QCOMPARE(CaptureEngine::chooseMode(std::nan(""),20000,4),2);
    }
    void nativeContinuousStopDrainsAndBurstRepeats()
    {
        QString path=qEnvironmentVariable("J_ESC_TEST_BRIDGE");if(path.isEmpty())QSKIP("Native firmware bridge missing");
        QLibrary lib(path);QVERIFY(lib.load());
        auto open=reinterpret_cast<int(*)()>(lib.resolve("test_bridge_open"));
        auto write=reinterpret_cast<void(*)(const uint8_t *,uint32_t)>(lib.resolve("test_bridge_write"));
        auto read=reinterpret_cast<uint32_t(*)(uint8_t *,uint32_t)>(lib.resolve("test_bridge_read"));
        auto ticks=reinterpret_cast<void(*)(uint32_t)>(lib.resolve("test_bridge_ticks"));
        auto commands=reinterpret_cast<int(*)()>(lib.resolve("test_bridge_commands"));
        QVERIFY(open && write && read && ticks && commands);QCOMPARE(open(),0);
        ProtocolSession session;auto buffer=std::make_shared<ScopeBuffer>();CaptureEngine capture(&session,buffer);
        connect(&session,&ProtocolSession::ready,&capture,&CaptureEngine::install);
        connect(&session,&ProtocolSession::failed,&capture,&CaptureEngine::disconnected);
        QTimer poll;int readLoops=128;bool overflowOnce=false;
        connect(&session,&ProtocolSession::outgoing,&poll,[&](const QByteArray &data){write(reinterpret_cast<const uint8_t *>(data.constData()),uint32_t(data.size()));});
        connect(&poll,&QTimer::timeout,&session,[&] {
            ticks(overflowOnce?20000:100);overflowOnce=false;
            for(int i=0;i<readLoops;++i){uint8_t bytes[97];auto size=read(bytes,97);if(!size)break;session.ingest(QByteArray(reinterpret_cast<char *>(bytes),size));}
        });
        QSignalSpy ready(&session,&ProtocolSession::ready),failed(&session,&ProtocolSession::failed);
        QVariantMap status;connect(&capture,&CaptureEngine::metrics,&poll,[&](const QVariantMap &s){status=s;});
        poll.start(1);session.start();QTRY_COMPARE(ready.count(),1);
        QVector<Entry> channels={channel(),channel(7,4,2)};
        capture.begin(channels,20000,1);QTRY_VERIFY(buffer->summary()["received"].toULongLong()>1000);
        capture.stop();QTRY_VERIFY(!capture.active());QCOMPARE(failed.count(),0);
        auto first=buffer->snapshot();QVERIFY(!first.isEmpty());QCOMPARE(first.front()->stride,quint16(8));
        for(const auto &b:first)for(int i=0;i<b->count;++i)QCOMPARE(b->values[2*i+1],7.0);
        capture.begin(channels,10000,2);QTRY_VERIFY(buffer->snapshot().size()>20);
        capture.stop();QTRY_VERIFY(!capture.active());QCOMPARE(failed.count(),0);QCOMPARE(commands(),0);
        auto bursts=buffer->snapshot();QVERIFY(bursts.back()->segment>bursts.front()->segment);
        capture.begin(channels,20000,2,true);QTRY_VERIFY(!capture.active());
        QCOMPARE(buffer->summary()["received"].toULongLong(),2048ULL);
        QCOMPARE(commands(),0);
        status.clear();
        capture.begin({channel()},10000,0);
        QTRY_VERIFY_WITH_TIMEOUT(status.value("state").toString()==QStringLiteral("采样中") ||
            !status.value("error").toString().isEmpty() || failed.count()>0,10000);
        QVERIFY2(failed.count()==0 && status.value("mode").toInt()==1,qPrintable(QString::fromUtf8(QJsonDocument::fromVariant(status).toJson())));
        overflowOnce=true;
        QTRY_VERIFY_WITH_TIMEOUT(status.value("mode").toInt()==2 && status.value("overflows").toInt()>0,10000);
        capture.stop();QTRY_VERIFY(!capture.active());QCOMPARE(failed.count(),0);QCOMPARE(commands(),0);
        capture.begin({channel()},10000,1);
        QTRY_VERIFY(buffer->summary()["received"].toULongLong()>0);
        auto last=buffer->snapshot().back();
        auto broken=blockBytes(last->tick+quint64(last->count)*last->divisor,last->sequence+2,last->segment,1,{channel()});
        od_wire_put32(reinterpret_cast<uint8_t *>(broken.data()),last->config);
        od_wire_put32(reinterpret_cast<uint8_t *>(broken.data()+24),last->divisor);
        session.stream(OD_MSG_SAMPLES,last->sequence+2,broken);
        QCOMPARE(failed.count(),1);QVERIFY(!capture.active());QCOMPARE(commands(),0);
        poll.stop();
    }
};
QTEST_GUILESS_MAIN(ScopeTests)
#include "test_scope.moc"
