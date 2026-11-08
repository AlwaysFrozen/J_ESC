#include "ScopeRecording.h"
#include "od_wire.h"
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonArray>
#include <QSaveFile>
#include <QSet>
#include <cstring>
#include <limits>
#include <algorithm>

namespace {
QByteArray u32(quint32 n) { QByteArray b(4,'\0');od_wire_put32(reinterpret_cast<uint8_t *>(b.data()),n);return b; }
QString format(quint16 type) {
    switch(type) {case 1:case 5:return "B";case 2:return "b";case 3:return "h";case 4:return "i";
    case 6:return "H";case 7:return "I";case 8:return "f";default:return {};}
}
int width(quint16 type) {switch(type){case 1:case 2:case 5:return 1;case 3:case 6:return 2;case 4:case 7:case 8:return 4;default:return 0;}}
QString quoted(QString s) {s.replace('"',"\"\"");return '"'+s+'"';}
}
ScopeRecorder::~ScopeRecorder(){finish(QStringLiteral("录制被关闭"));wait();}
bool ScopeRecorder::begin(const QString &path,const QJsonObject &meta,QString *error)
{
    if(isRunning()){*error=QStringLiteral("上次录制尚未完成写盘");return false;}
    wait();
    if(QFileInfo::exists(path) || QFileInfo::exists(path+".partial")){*error=QStringLiteral("录制文件或未完成文件已存在，请选择新文件");return false;}
    QMutexLocker lock(&m_mutex);
    auto json=QJsonDocument(meta).toJson(QJsonDocument::Compact);
    if(json.size()>1024*1024){*error=QStringLiteral("录制元数据过大");return false;}
    m_path=path;m_header="JESCOPE2\n"+u32(quint32(json.size()))+json;
    m_queue.clear();m_queued=0;m_written=0;m_closing=false;m_done=false;m_error.clear();start();return true;
}
bool ScopeRecorder::append(quint8 type,quint32 session,quint32 sequence,const QByteArray &payload)
{
    OD_WireFrame frame{};frame.type=type;frame.session=session;frame.sequence=sequence;
    if(payload.size()>OD_WIRE_MAX_PAYLOAD)return false;
    frame.length=quint16(payload.size());std::memcpy(frame.payload,payload.constData(),size_t(payload.size()));
    uint8_t bytes[OD_WIRE_ENCODED_SIZE];size_t size=od_wire_encode(&frame,bytes,sizeof(bytes));
    QByteArray record=u32(quint32(size))+QByteArray(reinterpret_cast<char *>(bytes),qsizetype(size));
    QMutexLocker lock(&m_mutex);
    if(m_done || m_closing || !m_error.isEmpty())return false;
    if(!size || m_queued+record.size()>m_budget) {
        m_error=QStringLiteral("写盘队列已满，录制不完整；已请求停止采集");m_closing=true;m_ready.wakeOne();return false;
    }
    m_queued+=record.size();m_queue.push_back(std::move(record));m_ready.wakeOne();return true;
}
void ScopeRecorder::finish(const QString &error)
{
    QMutexLocker lock(&m_mutex);if(m_done)return;
    if(m_error.isEmpty() && !error.isEmpty())m_error=error;
    m_closing=true;m_ready.wakeOne();
}
QVariantMap ScopeRecorder::status() const
{
    QMutexLocker lock(&m_mutex);
    return {{"recordPath",m_path},{"recordBusy",!m_done},{"recordBytes",QVariant::fromValue(m_written)},
        {"recordQueued",m_queued},{"recordError",m_error},{"recordComplete",m_done && m_error.isEmpty() && !m_path.isEmpty()}};
}
void ScopeRecorder::run()
{
    QFile file(m_path+".partial");
    auto write=[&](const QByteArray &data){
        if(file.write(data)!=data.size())return false;
        QMutexLocker lock(&m_mutex);m_written+=quint64(data.size());return true;
    };
    bool ok=file.open(QIODevice::WriteOnly|QIODevice::NewOnly);
    if(ok)ok=write(m_header);
    while(ok) {
        QByteArray record;
        {QMutexLocker lock(&m_mutex);
            while(m_queue.empty() && !m_closing)m_ready.wait(&m_mutex);
            if(m_queue.empty())break;
            record=std::move(m_queue.front());m_queue.pop_front();m_queued-=record.size();
        }
        ok=write(record);
    }
    if(ok)ok=file.flush();
    file.close();
    QMutexLocker lock(&m_mutex);
    if(!ok && m_error.isEmpty())m_error=QStringLiteral("写盘失败，录制不完整：%1").arg(file.errorString());
    if(ok && m_error.isEmpty() && !file.rename(m_path))m_error=QStringLiteral("录制无法完成重命名：%1").arg(file.errorString());
    m_queue.clear();m_queued=0;m_done=true;
}

QJsonObject ScopeRecording::metadata(const QVector<Entry> &channels,const QByteArray &caps,quint32 session,int rate,const QByteArray &hash)
{
    QJsonArray list;
    for(const auto &e:channels)list.append(QJsonObject{{"name",e.name},{"index",e.index},{"subindex",e.subindex},
        {"type",e.type},{"format",format(e.type)},{"size",e.size}});
    auto p=reinterpret_cast<const uint8_t *>(caps.constData());
    QJsonObject capabilities;
    if(caps.size()==18)capabilities={{"tick_hz",qint64(od_wire_u32(p))},{"slow_hz",qint64(od_wire_u32(p+4))},
        {"buffer_bytes",qint64(od_wire_u32(p+8))},{"block_bytes",od_wire_u16(p+12)},{"max_channels",p[14]},
        {"features",p[15]},{"max_payload",od_wire_u16(p+16)}};
    return {{"format_version",1},{"protocol_version",2},{"session",qint64(session)},{"sample_hz",rate},
        {"divisor",rate>0?capabilities["tick_hz"].toInt()/rate:0},{"capabilities",capabilities},{"channels",list},
        {"dictionary_sha256",QString::fromLatin1(hash)},{"dictionary_verified_by_device",false}};
}

bool ScopeRecording::read(const QString &path,std::shared_ptr<ScopeBuffer> buffer,const QString &csvPath,QVariantMap *result,QString *error,double windowEnd,double windowSpan)
{
    auto fail=[&](const QString &why){*error=why;return false;};
    if(path.endsWith(".partial",Qt::CaseInsensitive))return fail(QStringLiteral("这是未完成的录制，不能作为完整文件回放或导出"));
    QFile file(path);if(!file.open(QIODevice::ReadOnly))return fail(file.errorString());
    if(file.read(9)!="JESCOPE2\n")return fail(QStringLiteral("不是 JESCOPE2 记录"));
    auto prefix=file.read(4);if(prefix.size()!=4)return fail(QStringLiteral("记录头截断"));
    quint32 length=od_wire_u32(reinterpret_cast<const uint8_t *>(prefix.constData()));
    if(length>1024*1024)return fail(QStringLiteral("元数据过大"));
    auto json=file.read(length);QJsonParseError parseError;
    auto doc=QJsonDocument::fromJson(json,&parseError);
    if(json.size()!=length || parseError.error!=QJsonParseError::NoError || !doc.isObject())return fail(QStringLiteral("元数据无效或截断"));
    auto meta=doc.object();auto channelList=meta["channels"].toArray();
    if(meta["format_version"].toInt()!=1 || meta["protocol_version"].toInt()!=2 || channelList.isEmpty() || channelList.size()>16)
        return fail(QStringLiteral("不支持的记录版本或通道数"));
    QVector<Entry> channels;QStringList names;QSet<quint32> addresses;
    int stride=0;auto caps=meta["capabilities"].toObject();
    qint64 sessionValue=meta["session"].toInteger(-1),hzValue=caps["tick_hz"].toInteger(-1),divisorValue=meta["divisor"].toInteger(-1);
    if(sessionValue<=0 || sessionValue>UINT32_MAX || hzValue<=0 || hzValue>UINT32_MAX || divisorValue<=0 || divisorValue>UINT32_MAX ||
        hzValue%divisorValue || hzValue/divisorValue!=meta["sample_hz"].toInt())return fail(QStringLiteral("记录会话或采样频率无效"));
    quint32 session=quint32(sessionValue),hz=quint32(hzValue),divisor=quint32(divisorValue);
    for(const auto &value:channelList) {
        auto c=value.toObject();Entry e;e.name=c["name"].toString();int type=c["type"].toInt(-1);
        if(type<1 || type>65535)return fail(QStringLiteral("通道类型无效"));
        e.type=quint16(type);
        int index=c["index"].toInt(-1),sub=c["subindex"].toInt(-1),size=c["size"].toInt();
        if(e.name.isEmpty() || names.contains(e.name) || index<0x2100 || index>0x2109 || sub<1 || sub>255 ||
            !width(e.type) || size!=width(e.type) || (c["format"].toString()!=format(e.type) && c["format"].toString()!=QStringLiteral("<")+format(e.type)) || addresses.contains((quint32(index)<<8)|quint32(sub)))
            return fail(QStringLiteral("通道元数据无效"));
        e.index=quint16(index);e.subindex=quint8(sub);e.size=quint8(size);e.access="ro";
        addresses.insert((quint32(index)<<8)|quint32(sub));stride+=size;channels.append(e);names.append(e.name);
    }
    QSaveFile csv(csvPath);
    if(!csvPath.isEmpty()) {
        if(QFileInfo::exists(csvPath))return fail(QStringLiteral("CSV 已存在，请选择新文件"));
        if(!csv.open(QIODevice::WriteOnly))return fail(csv.errorString());
        QByteArray header="\xEF\xBB\xBFsegment,device_tick,device_time_s";
        for(const auto &name:names)header+=','+quoted(name).toUtf8();
        header+='\n';
        if(csv.write(header)!=header.size())return fail(csv.errorString());
    }
    // Validation uses constant memory even for long files. Segments are strictly ordered;
    // a final status may precede the last data blocks (the device's drain semantics).
    quint32 segment=0,config=0,next=0;quint64 tick=0,total=0,received=0,expected=0,segments=0;
    bool have=false,final=false;quint32 overflows=0;
    double firstTime=-1,lastTime=0;
    auto complete=[&]{return have && final && expected==received;};
    if(buffer)buffer->clear(int(channels.size()));
    while(!file.atEnd()) {
        prefix=file.read(4);if(prefix.size()!=4)return fail(QStringLiteral("记录长度截断"));
        length=od_wire_u32(reinterpret_cast<const uint8_t *>(prefix.constData()));
        if(!length || length>OD_WIRE_ENCODED_SIZE)return fail(QStringLiteral("记录帧长度无效"));
        auto bytes=file.read(length);
        if(bytes.size()!=length || bytes.back()!=0)return fail(QStringLiteral("记录帧截断"));
        OD_WireParser parser{};OD_WireFrame frame{};od_wire_parser_init(&parser);int decoded=0;
        for(char ch:bytes)if(od_wire_feed(&parser,quint8(ch),&frame))++decoded;
        if(decoded!=1 || parser.errors || parser.used || parser.discard || frame.session!=session)
            return fail(QStringLiteral("CRC、帧格式或会话校验失败"));
        QByteArray payload(reinterpret_cast<char *>(frame.payload),frame.length);
        quint32 incoming=0,incomingConfig=0;
        bool isFinal=frame.type==(OD_MSG_SCOPE_STATUS|OD_MSG_REPLY);
        if(frame.type==OD_MSG_SAMPLES && payload.size()>=32) {incoming=od_wire_u32(frame.payload+4);incomingConfig=od_wire_u32(frame.payload);}
        else if(isFinal && payload.size()==29 && frame.payload[0]==0) {incoming=od_wire_u32(frame.payload+9);incomingConfig=od_wire_u32(frame.payload+5);}
        else return fail(QStringLiteral("记录中含有不支持的帧"));
        if(!have || incoming!=segment) {
            if(have && (!complete() || incoming<=segment))return fail(QStringLiteral("上一段不完整或段号倒退"));
            if(have && csv.isOpen() && csv.write("\n",1)!=1)return fail(csv.errorString());
            have=true;segment=incoming;config=incomingConfig;next=0;received=0;final=false;++segments;
        }
        if(incomingConfig!=config)return fail(QStringLiteral("段内配置号变化"));
        if(isFinal) {
            auto p=frame.payload+1;
            if(final || p[0]!=0 || (p[1]!=1 && p[1]!=2) || p[2]!=channels.size() || p[3]!=0 ||
                od_wire_u32(p+20)!=divisor || od_wire_u16(p+24)!=stride || od_wire_u16(p+26)!=1024/stride)
                return fail(QStringLiteral("最终停止状态无效"));
            expected=od_wire_u32(p+12);overflows=std::max(overflows,od_wire_u32(p+16));final=true;
            if(received>expected)return fail(QStringLiteral("最终点数小于已收点数"));
            continue;
        }
        QString why;auto block=ScopeBlock::decode(payload,frame.sequence,channels,&why);
        if(!block || block->sequence!=next || block->hz!=hz || block->divisor!=divisor || (next && block->tick!=tick) ||
            (next==0 && total && block->tick<tick))return fail(QStringLiteral("样本序号、设备时间或通道布局错误：%1").arg(why));
        ++next;tick=block->tick+quint64(block->count)*divisor;received+=block->count;total+=block->count;
        if(firstTime<0)firstTime=block->start();
        lastTime=block->end();
        if(final && received>expected)return fail(QStringLiteral("样本超过最终点数"));
        if(buffer && (windowEnd<0 || (block->end()>=windowEnd-windowSpan && block->start()<=windowEnd)) && !buffer->append(block))
            return fail(QStringLiteral("采样块超过回放缓存"));
        if(csv.isOpen()) {
            QByteArray rows;
            for(int i=0;i<block->count;++i) {
                quint64 t=block->tick+quint64(i)*divisor;
                rows+=QByteArray::number(segment)+','+QByteArray::number(t)+','+QByteArray::number(double(t)/hz,'f',9);
                int offset=0;for(const auto &e:channels){rows+=','+e.decode(block->raw.mid(i*stride+offset,e.size)).toUtf8();offset+=e.size;}
                rows+='\n';
            }
            if(csv.write(rows)!=rows.size())return fail(csv.errorString());
        }
    }
    if(!complete())return fail(QStringLiteral("记录缺少最终状态或最终点数不匹配"));
    if(csv.isOpen() && !csv.commit())return fail(csv.errorString());
    *result={{"channelNames",names},{"received",QVariant::fromValue(total)},{"segments",QVariant::fromValue(segments)},
        {"overflows",overflows},{"rate",meta["sample_hz"].toInt()},{"path",path},{"replayStart",firstTime},{"replayEnd",lastTime}};
    return true;
}
