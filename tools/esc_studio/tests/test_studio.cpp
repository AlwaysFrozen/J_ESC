#include "Dictionary.h"
#include "ProtocolSession.h"
#include "ParameterScheme.h"
#include <QTemporaryDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QtTest>
#include <QTemporaryFile>
#include <QLibrary>
#include <cstring>

namespace {
QByteArray encode(const OD_WireFrame &frame)
{
    uint8_t out[OD_WIRE_ENCODED_SIZE]; size_t size=od_wire_encode(&frame,out,sizeof(out));
    return QByteArray(reinterpret_cast<char *>(out),qsizetype(size));
}
Entry scalar(quint16 type,quint8 size)
{ Entry e; e.name="value"; e.type=type; e.size=size; e.typeName="test"; return e; }
}

class StudioTests : public QObject {
    Q_OBJECT
private slots:
    void schemeRoundTripAtomicAndMetadata()
    {
        Dictionary d;QString error;QVERIFY(d.load(JESC_DICTIONARY_PATH,&error));ParameterScheme scheme(&d);QTemporaryDir dir;
        QUrl file=QUrl::fromLocalFile(dir.filePath("motor.jescconfig"));
        QVERIFY(!scheme.save(file));
        for(int g=0;g<7;++g)for(int row:d.rowsForGroup(g)){QByteArray b;QVERIFY(d.entry(row)->encode("0",&b,&error));d.acceptRead(row,b);}
        int rated=d.find("motor0_rated_voltage");QVERIFY(d.editValue(rated,"24"));QVERIFY(scheme.save(file));QVERIFY(!scheme.save(file));
        QVERIFY(d.editValue(rated,"12"));QVERIFY(scheme.preview(file));QCOMPARE(scheme.differences().size(),1);
        QVERIFY(scheme.apply());QCOMPARE(d.entry(rated)->draftText,QString("24"));
        QVERIFY(scheme.preview(file));QVERIFY(d.editValue(rated,"48"));QVERIFY(!scheme.apply());QCOMPARE(d.entry(rated)->draftText,QString("48"));
        QFile source(file.toLocalFile());QVERIFY(source.open(QIODevice::ReadOnly));auto root=QJsonDocument::fromJson(source.readAll()).object();
        root["dictionary_sha256"]="wrong";QFile bad(dir.filePath("bad.json"));QVERIFY(bad.open(QIODevice::WriteOnly));bad.write(QJsonDocument(root).toJson());bad.close();
        QVERIFY(!scheme.preview(QUrl::fromLocalFile(bad.fileName())));QCOMPARE(d.entry(rated)->draftText,QString("48"));
        QCOMPARE(d.entry(rated)->unit,QString("V"));auto options=d.entry(d.find("motor0_type"))->options;
        QCOMPARE(options.size(),2);QVERIFY(options[0].toMap()["label"].toString().startsWith("BLDC"));
    }
    void exactScalarConversion()
    {
        QByteArray data; QString error;
        Entry e=scalar(0x1B,8);
        QVERIFY(e.encode("18446744073709551615",&data,&error));
        QCOMPARE(e.decode(data),QString("18446744073709551615"));
        QVERIFY(!e.encode("18446744073709551616",&data,&error));
        QVERIFY(!e.encode("-1",&data,&error));
        e=scalar(0x15,8);
        QVERIFY(e.encode("-9223372036854775808",&data,&error));
        QCOMPARE(e.decode(data),QString("-9223372036854775808"));
        e=scalar(2,1);
        QVERIFY(e.encode("-0x80",&data,&error)); QCOMPARE(e.decode(data),QString("-128"));
        QVERIFY(!e.encode("128",&data,&error));
        e=scalar(8,4);
        QVERIFY(e.encode("12.5",&data,&error)); QCOMPARE(data.toHex(),QByteArray("00004841"));
        QVERIFY(!e.encode("NaN",&data,&error)); QVERIFY(!e.encode("1e300",&data,&error));
        e=scalar(1,1); QVERIFY(!e.encode("2",&data,&error));
    }
    void dictionaryGroupsAndDraftLifecycle()
    {
        Dictionary d; QString error;
        QVERIFY2(d.load(JESC_DICTIONARY_PATH,&error),qPrintable(error));
        QCOMPARE(d.rowCount(),626);
        const int counts[]={26,11,17,8,58,36,10};
        ParameterFilter filter; filter.setSourceModel(&d);
        for(int group=0;group<7;++group) {
            filter.setGroup(group); QCOMPARE(filter.rowCount(),counts[group]);
        }
        int row=d.find("motor0_rated_voltage"); QVERIFY(row>=0);
        QVERIFY(d.editValue(row,"12.5")); QCOMPARE(d.dirtyCount(),1);
        auto encoded=d.entry(row)->draft;
        d.acceptRead(row,QByteArray::fromHex("0000c041")); // Device is 24 V, preserve the local 12.5 V edit.
        QVERIFY(d.entry(row)->edited); QCOMPARE(d.entry(row)->draft,encoded);
        d.acceptWrite(row,encoded); QVERIFY(d.entry(row)->edited); // ACK is not readback.
        d.acceptRead(row,encoded); QVERIFY(!d.entry(row)->edited); QCOMPARE(d.dirtyCount(),0);
        QVERIFY(d.editValue(row,"10")); d.resetDevice(); QVERIFY(d.entry(row)->edited); QVERIFY(!d.entry(row)->known);
        QVERIFY(!d.editValue(d.find("system_save_flash"),"1"));
    }
    void transactionalDictionaryLoadAndQuotedCsv()
    {
        Dictionary d; QString error;
        QVERIFY(d.load(JESC_DICTIONARY_PATH,&error));
        QTemporaryFile file; QVERIFY(file.open());
        file.write("name,index,subindex,data_type,access,description\nvalue,0x2000,0x01,REAL32,rw,\"a,b\nsecond\"\n"); file.flush();
        QVERIFY2(d.load(file.fileName(),&error),qPrintable(error)); QCOMPARE(d.rowCount(),1);
        QCOMPARE(d.entry(0)->description,QString("a,b\nsecond"));
        file.resize(0); file.seek(0);
        file.write("name,index,subindex,data_type,access\nx,0x2000,0x01,REAL32,rw\nx,0x2001,0x01,REAL32,rw\n"); file.flush();
        QVERIFY(!d.load(file.fileName(),&error)); QCOMPARE(d.rowCount(),1); QCOMPARE(d.entry(0)->name,QString("value"));
    }
    void timeoutNeverRetries()
    {
        ProtocolSession session; QSignalSpy outgoing(&session,&ProtocolSession::outgoing),failed(&session,&ProtocolSession::failed);
        session.start(20); QTRY_COMPARE(failed.count(),1); QCOMPARE(outgoing.count(),1);
        QVERIFY(!session.isReady());
    }
    void cancelsUnexecutedBatchAfterWriteFailure()
    {
        ProtocolSession session; OD_WireParser parser; od_wire_parser_init(&parser);
        int writes=0;
        connect(&session,&ProtocolSession::outgoing,&session,[&](const QByteArray &packet) {
            if(packet.startsWith("!\n")) {
                QTimer::singleShot(0,&session,[&]{session.ingest("O 00000000 80 -\nO 00000001 00 02\n");}); return;
            }
            OD_WireFrame in;
            for(char ch:packet) if(od_wire_feed(&parser,quint8(ch),&in)) {
                OD_WireFrame response{}; response.type=in.type|OD_MSG_REPLY; response.session=9; response.sequence=in.sequence;
                response.length=1;
                if(in.type==OD_MSG_CAPABILITIES) {
                    response.length=19; od_wire_put32(response.payload+1,20000);
                    od_wire_put16(response.payload+17,OD_WIRE_MAX_PAYLOAD);
                } else { ++writes; response.payload[0]=0x0C; }
                QByteArray bytes=encode(response);
                QTimer::singleShot(0,&session,[&,bytes]{session.ingest(bytes);});
            }
        });
        QSignalSpy ready(&session,&ProtocolSession::ready),result(&session,&ProtocolSession::result);
        session.start(); QTRY_COMPARE(ready.count(),1);
        QByteArray payload=QByteArray::fromHex("00200108000400004841");
        session.enqueue(1,OD_MSG_WRITE,payload); session.enqueue(2,OD_MSG_WRITE,payload); session.enqueue(3,OD_MSG_WRITE,payload);
        QTRY_COMPARE(result.count(),3); QCOMPARE(writes,1);
        QCOMPARE(result[0][1].toInt(),0x0C); QCOMPARE(result[1][1].toInt(),0xFE); QCOMPARE(result[2][1].toInt(),0xFE);
    }
    void nativeFirmwareBridge()
    {
        const QString path=qEnvironmentVariable("J_ESC_TEST_BRIDGE");
        if(path.isEmpty()) QSKIP("Set J_ESC_TEST_BRIDGE to the C firmware test DLL to run interoperability");
        QLibrary library(path); QVERIFY2(library.load(),qPrintable(library.errorString()));
        auto open=reinterpret_cast<int(*)()>(library.resolve("test_bridge_open"));
        auto write=reinterpret_cast<void(*)(const uint8_t *,uint32_t)>(library.resolve("test_bridge_write"));
        auto read=reinterpret_cast<uint32_t(*)(uint8_t *,uint32_t)>(library.resolve("test_bridge_read"));
        auto ticks=reinterpret_cast<void(*)(uint32_t)>(library.resolve("test_bridge_ticks"));
        auto commands=reinterpret_cast<int(*)()>(library.resolve("test_bridge_commands"));
        QVERIFY(open && write && read && ticks && commands); QCOMPARE(open(),0);
        ProtocolSession session; QTimer poll;
        connect(&session,&ProtocolSession::outgoing,&poll,[&](const QByteArray &data){write(reinterpret_cast<const uint8_t *>(data.constData()),uint32_t(data.size()));});
        connect(&poll,&QTimer::timeout,&session,[&] {
            ticks(8);
            for(int i=0;i<64;++i) { uint8_t bytes[5]; auto count=read(bytes,5); if(!count) break; session.ingest(QByteArray(reinterpret_cast<char *>(bytes),count)); }
        });
        poll.start(1);
        QSignalSpy ready(&session,&ProtocolSession::ready),results(&session,&ProtocolSession::result),failed(&session,&ProtocolSession::failed);
        session.start(); QTRY_COMPARE(ready.count(),1); QVERIFY(session.isReady());
        Entry e=scalar(7,4); e.index=0x2100; e.subindex=2;
        session.enqueue(7,OD_MSG_READ,e.requestHeader()); QTRY_COMPARE(results.count(),1);
        QCOMPARE(results[0][0].toInt(),7); QCOMPARE(results[0][1].toInt(),0);
        QCOMPARE(results[0][2].toByteArray(),QByteArray::fromHex("07000000"));
        e.index=0; e.subindex=1;
        session.enqueue(8,OD_MSG_WRITE,e.requestHeader()+QByteArray::fromHex("00000000")); QTRY_COMPARE(results.count(),2);
        QCOMPARE(results[1][1].toInt(),0); QCOMPARE(commands(),1); QCOMPARE(failed.count(),0);
        poll.stop(); session.abort("test complete");
    }
};
QTEST_GUILESS_MAIN(StudioTests)
#include "test_studio.moc"
