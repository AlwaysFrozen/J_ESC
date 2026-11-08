#include "CaptureEngine.h"
#include <QSet>
#include <algorithm>
#include <cmath>

namespace {
struct Status {
    quint8 state=0,mode=0,channels=0;
    quint32 config=0,segment=0,captured=0,overflows=0,divisor=0;
    quint16 stride=0,perBlock=0;
    static bool decode(const QByteArray &bytes,Status *s) {
        if(bytes.size()!=28) return false;
        auto p=reinterpret_cast<const uint8_t *>(bytes.constData());
        if(p[0]>3 || p[3]!=0) return false;
        s->state=p[0];s->mode=p[1];s->channels=p[2];s->config=od_wire_u32(p+4);s->segment=od_wire_u32(p+8);
        s->captured=od_wire_u32(p+12);s->overflows=od_wire_u32(p+16);s->divisor=od_wire_u32(p+20);
        s->stride=od_wire_u16(p+24);s->perBlock=od_wire_u16(p+26);return true;
    }
};
}
CaptureEngine::CaptureEngine(ProtocolSession *session,std::shared_ptr<ScopeBuffer> buffer,QObject *parent)
    :QObject(parent),m_session(session),m_buffer(std::move(buffer)),m_timer(new QTimer(this))
{
    connect(session,&ProtocolSession::result,this,&CaptureEngine::result);
    connect(session,&ProtocolSession::stream,this,&CaptureEngine::stream);
    m_timer->setInterval(100);
    connect(m_timer,&QTimer::timeout,this,[this] {
        if(m_recording && !m_recorder.status().value("recordError").toString().isEmpty()) {
            m_error=m_recorder.status().value("recordError").toString();m_recording=false;stop();m_dirty=true;
        }
        if(active()) {
            int timeout=std::max(5000,int(2000.0*std::max(1U,m_memory)/(std::max(1,m_stride)*std::max(1,m_rate)))+2000);
            if(m_activity.isValid() && m_activity.elapsed()>timeout) { fail(QStringLiteral("采样数据停止到达，已断开会话")); }
            else if(m_phase==Capturing && m_mode==1 && m_operation==None && m_statusClock.elapsed()>=250)
                request(StatusAck,OD_MSG_SCOPE_STATUS);
        }
        if(m_dirty || m_recorder.status().value("recordBusy").toBool() || !m_recordPath.isEmpty()) notify();
    });
    m_timer->start();
}
bool CaptureEngine::active() const
{ return m_phase>=Benchmark && m_phase<=Draining; }
int CaptureEngine::chooseMode(double bps,double hz,int stride,int bytes)
{
    if(!std::isfinite(bps) || bps<=0 || hz<=0 || stride<=0 || stride>bytes) return 2;
    int useful=bytes/stride*stride;
    return hz*stride*(1.0+64.0/useful)<=bps*0.70 ? 1 : 2;
}
void CaptureEngine::install(const QByteArray &caps)
{
    m_caps=caps; m_operation=None;m_pending=0;m_started=false;
    if(caps.size()!=18) {m_phase=Error;m_error=QStringLiteral("能力描述长度无效");notify();return;}
    auto p=reinterpret_cast<const uint8_t *>(caps.constData());
    m_hz=od_wire_u32(p);m_memory=od_wire_u32(p+8);m_blockBytes=od_wire_u16(p+12);m_maxChannels=p[14];
    m_phase=Idle;m_error.clear();notify();
}
void CaptureEngine::disconnected(const QString &reason)
{
    if(active())completeRecording(reason);
    bool wasActive=active();m_phase=Offline;m_pending=0;m_operation=None;
    if(wasActive)m_error=reason;
    notify();
}
void CaptureEngine::begin(const QVector<Entry> &channels,int rate,int requested,bool single)
{
    if(active())return;
    if(m_recorder.isRunning()){m_error=QStringLiteral("请等待录制写盘完成");notify();return;}
    auto reject=[&](const QString &message){m_phase=Error;m_error=message;notify();};
    if(!m_session->isReady() || m_hz<10000) {reject(QStringLiteral("请先连接支持采样的设备"));return;}
    if(channels.isEmpty() || channels.size()>std::min(16,m_maxChannels) || rate<10000 ||
        m_hz%quint32(rate)!=0 || requested<0 || requested>2 || m_blockBytes!=1024 || m_memory!=16384) {
        reject(QStringLiteral("采样率、通道数或设备缓存能力不支持"));return;
    }
    int stride=0;QSet<quint32> addresses;
    for(const Entry &e:channels) {
        quint32 address=(quint32(e.index)<<8)|e.subindex;
        if(e.index<0x2100 || e.index>0x2109 || !e.subindex || e.access!="ro" ||
            (e.size!=1 && e.size!=2 && e.size!=4) || addresses.contains(address)) {
            reject(QStringLiteral("请选择不重复的 32 位及以下只读运行变量"));return;
        }
        addresses.insert(address);stride+=e.size;
    }
    m_channels=channels;m_stride=stride;m_rate=rate;m_divisor=m_hz/quint32(rate);m_requestedMode=requested;
    m_single=single;m_wantStop=false;m_started=false;m_switchBurst=false;m_error.clear();m_afterDrainError.clear();
    m_total=0;m_segments=0;m_overflows=0;m_measured=0;m_benchRound=0;
    m_operation=None;m_pending=0;m_buffer->clear(int(channels.size()));m_activity.start();m_totalClock.start();
    m_recording=false;m_repeatBurst=false;
    if(!m_recordPath.isEmpty()) {
        QString error;
        if(!m_recorder.begin(m_recordPath,ScopeRecording::metadata(channels,m_caps,m_session->sessionId(),rate,m_dictionaryHash),&error)) {reject(error);return;}
        m_recording=true;
    }
    if(requested==0 && !single && (quint8(m_caps[15])&8)) {
        m_phase=Benchmark;m_benchSequence=0;
        QByteArray count(2,'\0');od_wire_put16(reinterpret_cast<uint8_t *>(count.data()),256);
        request(BenchAck,OD_MSG_BENCHMARK,count);
    } else {m_mode=single?2:(requested==0?2:requested);configure();}
    m_dirty=true;
}
void CaptureEngine::request(Operation operation,quint8 type,const QByteArray &payload)
{
    if(m_operation!=None) {fail(QStringLiteral("采集命令状态冲突"));return;}
    if(m_token<-2000000000)m_token=-1000;
    m_operation=operation;m_pending=--m_token;
    m_session->enqueue(m_pending,type,payload,1500,type==OD_MSG_SCOPE_STOP);
}
void CaptureEngine::configure()
{
    if(m_wantStop){m_phase=Complete;completeRecording(QStringLiteral("未启动采样"));m_dirty=true;return;}
    m_phase=Configure;
    QByteArray payload(6,'\0');payload[0]=char(m_mode);payload[1]=char(m_channels.size());
    od_wire_put32(reinterpret_cast<uint8_t *>(payload.data()+2),m_divisor);
    for(const Entry &e:m_channels) {auto header=e.requestHeader();payload+=header.left(5);}
    request(ConfigAck,OD_MSG_SCOPE_CONFIG,payload);m_dirty=true;
}
void CaptureEngine::startSegment()
{
    if(m_wantStop){m_phase=Complete;completeRecording();m_dirty=true;return;}
    m_phase=Starting;m_received=0;m_nextBlock=0;m_nextTick=0;m_expected=0;
    request(StartAck,OD_MSG_SCOPE_START);m_dirty=true;
}
void CaptureEngine::stop()
{
    if(!active())return;
    m_wantStop=true;
    if(m_operation==None && m_phase!=Draining)sendStop();
    m_dirty=true;
}
void CaptureEngine::sendStop()
{
    m_phase=Stopping;request(StopAck,OD_MSG_SCOPE_STOP);m_dirty=true;
}
void CaptureEngine::result(int token,int code,const QByteArray &data)
{
    if(token!=m_pending || m_operation==None)return;
    Operation operation=m_operation;m_operation=None;m_pending=0;m_activity.restart();
    if(code!=0){fail(QStringLiteral("采集命令失败：0x%1").arg(code,2,16,QChar('0')));return;}
    if(operation==BenchAck) {
        if(m_wantStop){sendStop();return;}
        m_benchClock.start();return;
    }
    if(operation==StopAck) {
        if(!m_started){m_phase=Complete;completeRecording(QStringLiteral("未启动采样"));m_dirty=true;return;}
        request(FinalAck,OD_MSG_SCOPE_STATUS);return;
    }
    Status status;
    if(!Status::decode(data,&status) || status.stride!=m_stride || status.divisor!=m_divisor ||
        status.mode!=m_mode || status.channels!=m_channels.size() || status.perBlock!=m_blockBytes/m_stride) {
        fail(QStringLiteral("采集状态与通道配置不一致"));return;
    }
    if(operation==ConfigAck) {m_config=status.config;m_started=false;startSegment();return;}
    if(status.config!=m_config){fail(QStringLiteral("采集配置号变化"));return;}
    if(operation==StartAck) {
        m_segment=status.segment;m_baselineOverflow=status.overflows;m_started=true;++m_segments;
        m_phase=Capturing;m_statusClock.start();m_activity.restart();m_dirty=true;
        if(m_wantStop)sendStop();
        return;
    }
    if(status.segment!=m_segment || status.captured<m_received) {fail(QStringLiteral("采集点数或段号不一致"));return;}
    m_overflows=std::max(m_overflows,status.overflows);
    if(operation==FinalAck) {
        if(status.state!=0){fail(QStringLiteral("设备未确认停止采样"));return;}
        record(OD_MSG_SCOPE_STATUS|OD_MSG_REPLY,0,QByteArray(1,'\0')+data);
        m_expected=status.captured;m_phase=Draining;finishDrain();return;
    }
    m_statusClock.restart();
    if(status.state==3 || status.overflows>m_baselineOverflow) {
        m_switchBurst=m_requestedMode==0 && !m_single;
        if(!m_switchBurst)m_afterDrainError=QStringLiteral("连续采样缓存溢出，请减少通道或改用间断模式");
        sendStop();return;
    }
    if(m_mode==2 && m_received==quint64(m_memory/m_blockBytes)*(m_blockBytes/m_stride)) {
        if(status.state!=2 || status.captured!=m_received){fail(QStringLiteral("间断采集尚未冻结或数据不完整"));return;}
        m_repeatBurst=!m_wantStop && !m_single;
        sendStop();return;
    }
    if(m_wantStop)sendStop();
}
void CaptureEngine::finishDrain()
{
    m_dirty=true;
    if(m_received>m_expected){fail(QStringLiteral("上传点数超过设备最终点数"));return;}
    if(m_received!=m_expected)return;
    if(m_repeatBurst && !m_wantStop) {m_repeatBurst=false;startSegment();return;}
    if(m_switchBurst && !m_wantStop) {
        m_switchBurst=false;m_mode=2;m_started=false;configure();return;
    }
    m_phase=m_afterDrainError.isEmpty()?Complete:Error;
    if(!m_afterDrainError.isEmpty())m_error=m_afterDrainError;
    completeRecording();
}
void CaptureEngine::stream(int type,quint32 sequence,const QByteArray &payload)
{
    if(type==OD_MSG_BENCHMARK_DATA) {
        if(m_phase!=Benchmark || m_operation==BenchAck)return;
        if(sequence!=quint32(m_benchSequence) || payload.size()!=1024) {fail(QStringLiteral("吞吐测试数据不完整"));return;}
        ++m_benchSequence;m_activity.restart();
        if(m_benchSequence==256) {
            double seconds=m_benchClock.nsecsElapsed()/1e9;
            if(seconds<0.02){m_measured=0;m_mode=2;configure();return;}
            double measured=256.0*1024/seconds;
            m_measured=m_benchRound==0?measured:std::min(m_measured,measured);
            if(++m_benchRound<3) {
                m_benchSequence=0;QByteArray count(2,'\0');od_wire_put16(reinterpret_cast<uint8_t *>(count.data()),256);
                request(BenchAck,OD_MSG_BENCHMARK,count);
            } else {m_mode=chooseMode(m_measured,m_rate,m_stride,m_blockBytes);configure();}
        }
        return;
    }
    if(type!=OD_MSG_SAMPLES || !m_started || (m_phase!=Capturing && m_phase!=Stopping && m_phase!=Draining))return;
    QString error;auto block=ScopeBlock::decode(payload,sequence,m_channels,&error);
    if(!block || block->config!=m_config || block->segment!=m_segment || block->sequence!=m_nextBlock ||
        block->hz!=m_hz || block->divisor!=m_divisor || (m_nextBlock && block->tick!=m_nextTick)) {
        fail(error.isEmpty()?QStringLiteral("样本配置、序号或段内时间出现缺口"):error);return;
    }
    m_nextTick=block->tick+quint64(block->count)*m_divisor;++m_nextBlock;
    m_received+=block->count;m_total+=block->count;m_activity.restart();
    record(OD_MSG_SAMPLES,sequence,payload);
    if(!m_buffer->append(block)){fail(QStringLiteral("单个采样块超过历史缓存容量"));return;}
    if(m_phase==Draining){finishDrain();return;}
    if(m_mode==2 && m_phase==Capturing && m_operation==None &&
        m_received==quint64(m_memory/m_blockBytes)*(m_blockBytes/m_stride))request(StatusAck,OD_MSG_SCOPE_STATUS);
    m_dirty=true;
}
void CaptureEngine::fail(const QString &why)
{
    completeRecording(why);
    m_error=why;m_phase=Error;m_operation=None;m_pending=0;m_dirty=true;
    // A broken acquisition must not leave an unknown stream running on the device.
    m_session->abort(why);notify();
}
void CaptureEngine::notify()
{
    static const QStringList names={"未连接","就绪","测量带宽","配置采样","启动采样","采样中","正在停止","排空数据","已停止","采集错误"};
    QStringList channelNames;for(const Entry &e:m_channels)channelNames.append(e.name);
    QVariantMap values={{"state",names[m_phase]},{"active",active()},{"mode",m_mode},{"rate",m_rate},
        {"received",QVariant::fromValue(m_total)},{"segments",m_segments},{"overflows",m_overflows},
        {"throughput",m_measured},{"error",m_error},{"channelNames",channelNames},
        {"elapsed",m_totalClock.isValid()?m_totalClock.elapsed()/1000.0:0}};
    if(!m_recordPath.isEmpty()) {auto recordStatus=m_recorder.status();for(auto it=recordStatus.begin();it!=recordStatus.end();++it)values[it.key()]=it.value();}
    emit metrics(values);
    m_dirty=false;
}
void CaptureEngine::record(quint8 type,quint32 sequence,const QByteArray &data)
{
    if(m_recording && !m_recorder.append(type,m_session->sessionId(),sequence,data)) {
        m_error=m_recorder.status().value("recordError").toString();
        m_recording=false;m_wantStop=true;m_dirty=true;
    }
}
void CaptureEngine::completeRecording(const QString &error)
{if(m_recording){m_recorder.finish(error);m_recording=false;}}
