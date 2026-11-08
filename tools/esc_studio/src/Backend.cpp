#include "Backend.h"
#include "ProtocolSession.h"
#include "CaptureEngine.h"
#include "DemoScope.h"
#include <QDateTime>
#include <QCryptographicHash>
#include <QFile>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QUrl>
#include <algorithm>

namespace {
struct Command { const char *key,*name; quint16 index; quint8 sub; quint32 value; const char *message; };
const Command commands[] = {
    {"enable","control0_enable",0,1,1,"已使能，尚未启动运动"},
    {"disable","control0_enable",0,1,0,"已失能"},
    {"align","control0_align",0,2,1,"寻相已启动，请观察状态确认完成"},
    {"identify","control0_identify",0,3,1,"辨识已启动，请观察状态确认完成"},
    {"fault_reset","system_fault_reset",0x1000,1,1,"故障复位已执行，请核对状态"},
    {"apply","system_apply_config",0x1000,3,1,"RAM 参数已应用，请核对校准状态"},
    {"save","system_save_flash",0x1000,2,1,"Flash 保存成功"}
};
const Command *findCommand(const QString &key)
{ for(const auto &c:commands) if(key==QLatin1String(c.key)) return &c; return nullptr; }
struct Status { const char *name,*label; };
const Status statuses[] = {
    {"motor_status0_motor_state","电机状态"},{"motor_status0_is_enabled","使能"},
    {"motor_status0_is_calibrated","采样校准"},{"motor_status0_is_aligned","寻相"},
    {"motor_status0_is_error","故障"},{"motor_status0_motor_ctrl_mode","控制模式"},
    {"motor_status0_err_code","故障码"},{"motor_status0_ext_err_code","扩展故障"},
    {"foc_status0_align_state","寻相阶段"},{"control0_identify_state","辨识阶段"},
    {"system_flash_status","Flash 状态"},{"system_config_pending","参数待应用"}
};
QString errorText(int status)
{
    switch(status) {
    case 0x0C: return QStringLiteral("设备状态忙，请核对使能、校准与当前模式");
    case 0x0D: return QStringLiteral("固件不支持此操作");
    case 0x0E: return QStringLiteral("执行失败，请读取故障与 Flash 状态");
    case 0x0F: return QStringLiteral("配置无效或尚未应用");
    case 0x81: return QStringLiteral("写参数前须停机失能");
    case 0xFE: return QStringLiteral("请求未执行或已取消");
    default: return QStringLiteral("设备返回 0x%1").arg(status,2,16,QChar('0'));
    }
}
}

class ConnectionWorker : public QObject {
    Q_OBJECT
public:
    explicit ConnectionWorker(std::shared_ptr<ScopeBuffer> buffer,QObject *parent=nullptr)
        :QObject(parent),session(new ProtocolSession(this)),capture(new CaptureEngine(session,std::move(buffer),this)),demoScope(new DemoScope(this))
    {
        connect(session,&ProtocolSession::outgoing,this,[this](const QByteArray &packet) {
            if(demo) { QTimer::singleShot(0,this,[this,packet]{ demoWrite(packet); }); return; }
            if(!port || port->write(packet)!=packet.size()) session->abort(QStringLiteral("串口短写，结果未知，请重新连接核对"));
        });
        connect(session,&ProtocolSession::ready,capture,&CaptureEngine::install);
        connect(session,&ProtocolSession::ready,this,&ConnectionWorker::connected);
        connect(session,&ProtocolSession::result,this,[this](int token,int code,const QByteArray &data){if(token>=0)emit result(token,code,data);});
        connect(session,&ProtocolSession::failed,this,[this](const QString &why) { closePort();capture->disconnected(why);emit lost(why); });
        connect(capture,&CaptureEngine::metrics,this,&ConnectionWorker::scopeMetrics);
        connect(demoScope,&DemoScope::packet,session,&ProtocolSession::ingest);
        auto timer=new QTimer(this);timer->setInterval(100);
        connect(timer,&QTimer::timeout,this,[this]{
            if(motionOwned && !motionPoll && session->isReady()) {
                motionPoll=true;session->enqueue(-12,OD_MSG_MOTION_HEARTBEAT,{},1500,true);
                session->enqueue(-10,OD_MSG_MOTION_STATUS,{},1500,true);
            }
        });timer->start();
        connect(session,&ProtocolSession::ready,this,[this](const QByteArray &caps){motionSupported=caps.size()==18 && (quint8(caps[15])&16);emit motionInfo({{"supported",motionSupported},{"state",QStringLiteral("未提交")}});});
        connect(session,&ProtocolSession::result,this,[this](int token,int code,const QByteArray &data){
            if(token<-13 || token>-10)return;
            if(token==-10)motionPoll=false;
            if(code){motionOwned=false;emit motionInfo({{"supported",motionSupported},{"state",QStringLiteral("请求被拒绝")},{"error",errorText(code)}});return;}
            if(token==-11 || token==-10) {
                if(data.size()!=16){session->abort(QStringLiteral("运动状态长度错误"));return;}
                int phase=quint8(data[4]);motionOwned=phase>=1 && phase<=3;
                const QStringList states={"已停止","已接收","已生效 / 保持","位置已到达","执行失败","超时或状态变化后失能"};
                emit motionInfo({{"supported",motionSupported},{"active",motionOwned},{"state",states.value(phase,"未知状态")},
                    {"id",od_wire_u32(reinterpret_cast<const uint8_t *>(data.constData()))},{"mode",quint8(data[6])},{"error",quint8(data[5])?errorText(quint8(data[5])):QString()}});
            }
            if(token==-13){motionOwned=false;emit motionInfo({{"supported",motionSupported},{"state",QStringLiteral("已请求停止")}});}
        });
    }
    ProtocolSession *session;
    CaptureEngine *capture;
    DemoScope *demoScope;
    bool motionSupported=false,motionOwned=false,motionPoll=false;
    void motionRequest(int kind,const QByteArray &payload={}) {
        if(!motionSupported || !session->isReady())return;
        session->enqueue(kind==OD_MSG_MOTION_SUBMIT?-11:-13,quint8(kind),payload,1500,true);
    }
    void open(const QString &name,bool simulated,const QString &dictionary)
    {
        closePort(); demo=simulated;
        if(demo) {
            Dictionary model; QString error;
            if(!model.load(dictionary,&error)) { emit lost(error); return; }
            demoEntries.clear(); demoNames.clear();
            for(int i=0;i<model.rowCount();++i) {
                Entry e=*model.entry(i); e.encode("0",&e.device,&error);
                demoEntries.insert((quint32(e.index)<<8)|e.subindex,e);
                demoNames.insert(e.name,(quint32(e.index)<<8)|e.subindex);
            }
            demoSet("motor_status0_motor_state",2); demoSet("motor_status0_is_calibrated",1);
            od_wire_parser_init(&demoParser);demoScope->reset();session->start();return;
        }
        port=new QSerialPort(this); port->setPortName(name); port->setBaudRate(115200);
        port->setDataBits(QSerialPort::Data8); port->setParity(QSerialPort::NoParity);
        port->setStopBits(QSerialPort::OneStop); port->setFlowControl(QSerialPort::NoFlowControl);
        port->setReadBufferSize(256*1024);
        connect(port,&QSerialPort::readyRead,this,[this]{ session->ingest(port->readAll()); });
        connect(port,&QSerialPort::errorOccurred,this,[this](QSerialPort::SerialPortError code) {
            if(code!=QSerialPort::NoError && port) session->abort(QStringLiteral("串口错误：%1").arg(port->errorString()));
        });
        if(!port->open(QIODevice::ReadWrite)) { QString why=port->errorString(); closePort(); emit lost(why); return; }
        port->setDataTerminalReady(true); port->setRequestToSend(true); port->clear();
        session->start();
    }
    void close()
    {
        session->abort(QStringLiteral("已断开连接；未自动改变电机运动状态"));
        closePort();
    }
signals:
    void connected(QByteArray capabilities);
    void result(int token,int status,QByteArray bytes);
    void lost(QString reason);
    void scopeMetrics(QVariantMap info);
    void motionInfo(QVariantMap info);
private:
    QSerialPort *port=nullptr;
    bool demo=false;
    QHash<quint32,Entry> demoEntries;
    QHash<QString,quint32> demoNames;
    OD_WireParser demoParser{};
    quint32 demoMotionId=0;
    int demoMotionPhase=0,demoMotionMode=0;
    QElapsedTimer demoLease;
    void closePort()
    {
        demoScope->shutdown();
        motionOwned=false;motionSupported=false;motionPoll=false;demoMotionPhase=0;
        if(port) { port->disconnect(this); port->setDataTerminalReady(false); port->close(); port->deleteLater(); port=nullptr; }
        demo=false;
    }
    void demoSet(const QString &name,quint32 value)
    {
        if(!demoNames.contains(name)) return;
        Entry &e=demoEntries[demoNames[name]]; QString error; e.encode(QString::number(value),&e.device,&error);
    }
    bool demoFlag(const QString &name) const
    { return demoNames.contains(name) && demoEntries[demoNames[name]].decode(demoEntries[demoNames[name]].device)!="0"; }
    void demoWrite(const QByteArray &packet)
    {
        if(!demo) return;
        if(packet.startsWith("!\nB ")) { session->ingest("O 00000000 80 -\nO 00000001 00 02\n"); return; }
        OD_WireFrame in;
        for(char ch:packet) if(od_wire_feed(&demoParser,quint8(ch),&in)) {
            OD_WireFrame out{}; out.type=in.type|OD_MSG_REPLY; out.sequence=in.sequence; out.session=0xD3000001;
            out.length=1; out.payload[0]=0;
            if(in.type==OD_MSG_CAPABILITIES) {
                out.length=19; od_wire_put32(out.payload+1,20000); od_wire_put32(out.payload+5,10000);
                od_wire_put32(out.payload+9,16384); od_wire_put16(out.payload+13,1024);
                out.payload[15]=16; out.payload[16]=31; od_wire_put16(out.payload+17,OD_WIRE_MAX_PAYLOAD);
            } else if(in.type>=OD_MSG_MOTION_SUBMIT && in.type<=OD_MSG_MOTION_STOP) {
                if(in.type==OD_MSG_MOTION_SUBMIT) {
                    if(in.length!=20 || !demoFlag("motor_status0_is_enabled") || !demoFlag("motor_status0_is_aligned"))out.payload[0]=0x0C;
                    else {++demoMotionId;demoMotionMode=in.payload[0];demoMotionPhase=2;demoLease.restart();
                        demoSet("motor_status0_motor_ctrl_mode",demoMotionMode==1?2:(demoMotionMode==2?4:5));}
                }
                if(in.type==OD_MSG_MOTION_HEARTBEAT)demoLease.restart();
                if(in.type==OD_MSG_MOTION_STOP || (demoMotionPhase==2 && demoLease.elapsed()>500)) {
                    demoMotionPhase=in.type==OD_MSG_MOTION_STOP?0:5;demoSet("motor_status0_is_enabled",0);
                    demoSet("motor_status0_motor_ctrl_mode",0);demoSet("motor_status0_motor_state",2);}
                if(!out.payload[0] && (in.type==OD_MSG_MOTION_STATUS || in.type==OD_MSG_MOTION_SUBMIT)) {
                    out.length=17;od_wire_put32(out.payload+1,demoMotionId);out.payload[5]=uint8_t(demoMotionPhase);out.payload[7]=uint8_t(demoMotionMode);
                    od_wire_put32(out.payload+9,demoMotionPhase==2?(demoMotionMode==1?2:(demoMotionMode==2?4:5)):0);
                }
            } else if(demoScope->handle(in,&out,demoEntries)) {
                // Demonstration stream uses the same decoder/state machine as USB.
            } else if(in.type==OD_MSG_READ || in.type==OD_MSG_WRITE) {
                quint32 key=(quint32(od_wire_u16(in.payload))<<8)|in.payload[2];
                if(in.length<6 || !demoEntries.contains(key)) out.payload[0]=2;
                else {
                    Entry &e=demoEntries[key];
                    if(e.type!=od_wire_u16(in.payload+3) || e.size!=in.payload[5]) out.payload[0]=6;
                    else if(in.type==OD_MSG_READ) {
                        if(e.access=="wo") out.payload[0]=4;
                        else { out.length=quint16(1+e.size); std::copy(e.device.begin(),e.device.end(),out.payload+1); }
                    } else if(e.access=="ro") out.payload[0]=4;
                    else if(e.access=="rw") {
                        if(demoFlag("motor_status0_is_enabled")) out.payload[0]=0x81;
                        else { e.device=QByteArray(reinterpret_cast<char *>(in.payload+6),e.size); demoSet("system_config_pending",1); }
                    } else if(e.name=="control0_enable") {
                        quint32 value=od_wire_u32(in.payload+6);
                        if(value && demoFlag("system_config_pending")) out.payload[0]=0x0F;
                        else { demoSet("motor_status0_is_enabled",value?1:0); demoSet("motor_status0_motor_state",value?3:2); }
                    } else if(e.name=="system_apply_config") {
                        if(demoFlag("motor_status0_is_enabled")) out.payload[0]=0x0C;
                        else { demoSet("system_config_pending",0); demoSet("motor_status0_is_aligned",0); }
                    } else if(e.name=="system_save_flash") {
                        if(demoFlag("motor_status0_is_enabled") || demoFlag("system_config_pending")) out.payload[0]=0x0C;
                    } else if(e.name=="control0_align") {
                        if(!demoFlag("motor_status0_is_enabled")) out.payload[0]=0x0C;
                        else demoSet("motor_status0_is_aligned",1);
                    }
                }
            } else if(in.type!=OD_MSG_LEAVE) out.payload[0]=0x0D;
            uint8_t data[OD_WIRE_ENCODED_SIZE]; size_t size=od_wire_encode(&out,data,sizeof(data));
            session->ingest(QByteArray(reinterpret_cast<const char *>(data),qsizetype(size)));
        }
    }
};

Backend::Backend(QObject *parent):QObject(parent),m_dictionary(this),m_parameters(this),
    m_buffer(std::make_shared<ScopeBuffer>()),m_scope(new ScopeViewModel(m_buffer,this)),m_worker(new ConnectionWorker(m_buffer))
{
    m_parameters.setSourceModel(&m_dictionary);
    QString error; if(!m_dictionary.load(":/canopen/object_dictionary.csv",&error)) log(error);
    m_scope->setDictionary(m_dictionary);
    connect(&m_dictionary,&Dictionary::validationError,this,&Backend::log);
    m_worker->moveToThread(&m_thread);
    connect(&m_thread,&QThread::finished,m_worker,&QObject::deleteLater);
    auto *worker=static_cast<ConnectionWorker *>(m_worker);
    connect(m_scope,&ScopeViewModel::recordRequested,this,[this,worker](const QString &path){
        QFile file(m_dictionary.source());QByteArray hash;
        if(file.open(QIODevice::ReadOnly))hash=QCryptographicHash::hash(file.readAll(),QCryptographicHash::Sha256).toHex();
        QMetaObject::invokeMethod(worker,[worker,path,hash]{worker->capture->setRecording(path,hash);},Qt::QueuedConnection);
    });
    connect(worker,&ConnectionWorker::connected,this,[this](const QByteArray &caps) {
        m_connected=true; m_connecting=false;
        m_scope->setConnected(true);
        m_readOnConnect=true;
        log(QStringLiteral("%1 · 高频控制 %2 Hz；字典版本尚未由设备核对").arg(m_label).arg(od_wire_u32(reinterpret_cast<const uint8_t *>(caps.constData()))));
        emit stateChanged(); refreshStatus();
    });
    connect(worker,&ConnectionWorker::result,this,&Backend::receive);
    connect(worker,&ConnectionWorker::scopeMetrics,m_scope,&ScopeViewModel::acceptMetrics);
    connect(worker,&ConnectionWorker::motionInfo,this,[this](const QVariantMap &v){m_motion=v;emit statusChanged();});
    connect(m_scope,&ScopeViewModel::statusChanged,this,&Backend::stateChanged);
    connect(m_scope,&ScopeViewModel::startRequested,this,[this,worker](const QVector<Entry> &channels,int rate,int mode,bool single) {
        for(const auto &pending:m_pending)if(pending.command!="@status") {m_scope->reject(QStringLiteral("请等待参数操作结束"));return;}
        if(!m_connected){m_scope->reject(QStringLiteral("请先连接设备"));return;}
        QMetaObject::invokeMethod(worker,[worker,channels,rate,mode,single]{worker->capture->begin(channels,rate,mode,single);},Qt::QueuedConnection);
    });
    connect(m_scope,&ScopeViewModel::stopRequested,this,[worker]{QMetaObject::invokeMethod(worker,[worker]{worker->capture->stop();},Qt::QueuedConnection);});
    connect(worker,&ConnectionWorker::lost,this,[this](const QString &reason) {
        m_motion={{"supported",false},{"state",QStringLiteral("已断开")}};
        m_connected=false; m_connecting=false; m_pending.clear(); m_dictionary.resetDevice();
        m_scope->setConnected(false);
        m_readOnConnect=false; m_batchLabel.clear();
        m_label=QStringLiteral("未连接"); log(reason); emit stateChanged(); emit statusChanged();
    });
    m_thread.start();
    m_poll.setInterval(1000); connect(&m_poll,&QTimer::timeout,this,&Backend::refreshStatus); m_poll.start();
    refreshPorts(); log(QStringLiteral("就绪。选择端口，或打开演示设备预览配置流程。"));
}
Backend::~Backend()
{
    auto *worker=static_cast<ConnectionWorker *>(m_worker);
    QMetaObject::invokeMethod(worker,[worker]{ worker->close(); },Qt::BlockingQueuedConnection);
    m_thread.quit(); m_thread.wait();
}
void Backend::log(const QString &text)
{
    m_message=text; m_log.prepend(QTime::currentTime().toString("HH:mm:ss")+"  "+text);
    while(m_log.size()>80) m_log.removeLast();
    emit messageChanged();
}
void Backend::refreshPorts()
{
    m_ports.clear(); for(const auto &info:QSerialPortInfo::availablePorts()) m_ports.append(info.portName());
    m_ports.sort(); emit portsChanged();
}
void Backend::startConnection(const QString &port,bool demo)
{
    if(m_connected || m_connecting) return;
    m_connecting=true; m_label=demo ? QStringLiteral("演示设备 · 本地模拟") : port;
    m_dictionary.resetDevice(); emit stateChanged(); emit statusChanged();
    auto *worker=static_cast<ConnectionWorker *>(m_worker); QString path=m_dictionary.source();
    QMetaObject::invokeMethod(worker,[worker,port,demo,path]{ worker->open(port,demo,path); },Qt::QueuedConnection);
}
void Backend::connectPort(const QString &port)
{ if(port.isEmpty()) log(QStringLiteral("请先选择 COM 端口")); else startConnection(port,false); }
void Backend::openDemo() { startConnection({},true); }
void Backend::disconnectPort()
{
    auto *worker=static_cast<ConnectionWorker *>(m_worker);
    QMetaObject::invokeMethod(worker,[worker]{ worker->close(); },Qt::QueuedConnection);
}
bool Backend::loadDictionary(const QUrl &url)
{
    if(m_connected || m_connecting || busy() || m_dictionary.dirtyCount()) { log(QStringLiteral("请先断开设备并处理本地修改，再载入字典")); return false; }
    QString error;
    if(!m_dictionary.load(url.toLocalFile(),&error)) { log(error); return false; }
    m_scope->setDictionary(m_dictionary);
    log(QStringLiteral("已载入对象字典，未向设备写入参数")); emit statusChanged(); return true;
}
void Backend::queue(int row,bool write,const QByteArray &value,const QString &command,bool priority)
{
    const Entry *e=m_dictionary.entry(row);
    if(!m_connected || !e || m_pending.size()>=200) return;
    if((write && e->access=="ro") || (!write && e->access=="wo") || (write && value.size()!=e->size)) return;
    int token=++m_nextToken;
    m_pending.insert(token,{row,write,value,command});
    QByteArray payload=e->requestHeader(); if(write) payload+=value;
    auto *worker=static_cast<ConnectionWorker *>(m_worker);
    int timeout=command=="save" ? 10000 : 1500;
    QMetaObject::invokeMethod(worker,[worker,token,write,payload,timeout,priority] {
        worker->session->enqueue(token,write?OD_MSG_WRITE:OD_MSG_READ,payload,timeout,priority);
    },Qt::QueuedConnection);
    emit stateChanged();
}
void Backend::readGroup(int group)
{
    if(busy() || !m_connected) return;
    m_batchLabel=QStringLiteral("读取配置组"); m_batchFailures=0;
    for(int row:m_dictionary.rowsForGroup(group)) queue(row);
    log(QStringLiteral("正在读取当前配置组"));
}
void Backend::writeGroup(int group)
{
    if(busy() || !m_connected) return;
    auto rows=m_dictionary.rowsForGroup(group,true);
    if(rows.isEmpty()) { log(QStringLiteral("当前组没有本地修改")); return; }
    m_batchLabel=QStringLiteral("写入并回读配置组"); m_batchFailures=0;
    for(int row:rows) queue(row,true,m_dictionary.entry(row)->draft);
    log(QStringLiteral("写入 %1 个修改项；失败后取消尚未执行的请求").arg(rows.size()));
}
void Backend::readRow(int row) { if(!busy()) queue(row); }
void Backend::submitMotion(int mode,const QString &target,const QString &speed,const QString &acc,const QString &jerk)
{
    if(!m_connected || busy() || !m_motion.value("supported").toBool())return;
    if(mode<1 || mode>3)return;
    QByteArray payload(4,'\0');payload[0]=char(mode);
    Entry e;e.type=8;e.size=4;e.name=QStringLiteral("运动目标");
    for(const QString &text:QStringList{target,mode==3?speed:"0",mode==1?"0":acc,mode==1?"0":jerk}) {
        QByteArray b;QString error;if(!e.encode(text,&b,&error)){log(error);return;}payload+=b;
    }
    auto worker=static_cast<ConnectionWorker *>(m_worker);
    QMetaObject::invokeMethod(worker,[worker,payload]{worker->motionRequest(OD_MSG_MOTION_SUBMIT,payload);},Qt::QueuedConnection);
}
void Backend::stopMotion()
{
    auto worker=static_cast<ConnectionWorker *>(m_worker);
    QMetaObject::invokeMethod(worker,[worker]{worker->motionRequest(OD_MSG_MOTION_STOP);},Qt::QueuedConnection);
}
void Backend::readAll()
{
    if(busy() || !m_connected)return;
    m_batchLabel=QStringLiteral("读取全部参数");m_batchFailures=0;
    for(int g=0;g<7;++g)for(int row:m_dictionary.rowsForGroup(g))queue(row);
}
void Backend::writeAllChanges()
{
    if(busy() || !m_connected)return;
    m_batchLabel=QStringLiteral("写入方案修改项");m_batchFailures=0;
    for(int g=0;g<7;++g)for(int row:m_dictionary.rowsForGroup(g,true))queue(row,true,m_dictionary.entry(row)->draft);
}
void Backend::refreshStatus()
{
    if(!m_connected || !m_pending.isEmpty()) return;
    for(const auto &status:statuses) { int row=m_dictionary.find(QLatin1String(status.name)); if(row>=0) queue(row,false,{},"@status"); }
}
bool Backend::commandAvailable(const QString &key) const
{
    const Command *c=findCommand(key); if(!c) return false;
    const Entry *e=m_dictionary.entry(m_dictionary.find(QLatin1String(c->name)));
    return e && e->index==c->index && e->subindex==c->sub && e->type==7 && e->size==4 && e->access=="wo";
}
void Backend::command(const QString &key,int identify)
{
    if(!m_connected || (busy() && key!="disable") || !commandAvailable(key)) return;
    const Command *c=findCommand(key);
    quint32 value=c->value;
    if(key=="identify") { if(identify<1 || identify>4) return; value=quint32(identify); }
    QByteArray bytes(4,'\0'); od_wire_put32(reinterpret_cast<uint8_t *>(bytes.data()),value);
    queue(m_dictionary.find(QLatin1String(c->name)),true,bytes,key,key=="disable");
    log(QStringLiteral("命令已排队，等待设备确认"));
}
void Backend::receive(int token,int status,const QByteArray &data)
{
    if(!m_pending.contains(token)) return;
    Pending pending=m_pending.take(token);
    if(status!=0) {
        if(!m_batchLabel.isEmpty() && pending.command!="@status") ++m_batchFailures;
        m_dictionary.fail(pending.row,errorText(status));
        if(status!=0xFE) log(errorText(status));
    } else if(pending.write && pending.command.isEmpty()) {
        m_dictionary.acceptWrite(pending.row,pending.bytes);
        queue(pending.row); // Confirmation is a separate read, never an assumed write result.
    } else if(pending.write) {
        const Command *c=findCommand(pending.command);
        if(c) log(QString::fromUtf8(c->message));
        if(pending.command=="apply") {
            // Applied device values remain separate from any unsent local edits.
            log(QStringLiteral("参数已应用到 RAM；Flash 需要单独保存"));
        }
    } else m_dictionary.acceptRead(pending.row,data);
    emit stateChanged(); emit statusChanged();
    if(m_pending.isEmpty()) {
        bool batchDone=!m_batchLabel.isEmpty();
        if(batchDone) {
            log(m_batchFailures ? QStringLiteral("%1完成：%2 项失败或取消，请检查各行状态").arg(m_batchLabel).arg(m_batchFailures)
                : QStringLiteral("%1完成").arg(m_batchLabel));
            m_batchLabel.clear();
        }
        if(m_readOnConnect) { m_readOnConnect=false; readGroup(m_parameters.group()); }
        else if(pending.write || batchDone) refreshStatus();
    }
}
QVariantList Backend::motorStatus() const
{
    QVariantList result;
    for(const auto &status:statuses) {
        const Entry *e=m_dictionary.entry(m_dictionary.find(QLatin1String(status.name)));
        QString value=e && e->known ? e->decode(e->device) : QStringLiteral("—");
        if(e && e->known && e->name=="motor_status0_motor_state") {
            const QStringList states={"初始化","采样校准","空闲","运行","故障"};
            bool ok; int state=value.toInt(&ok); if(ok && state>=0 && state<states.size()) value=states[state];
        }
        result.append(QVariantMap{{"label",QString::fromUtf8(status.label)},{"value",value}});
    }
    return result;
}
QString Backend::configState() const
{
    const Entry *e=m_dictionary.entry(m_dictionary.find("system_config_pending"));
    if(!e || !e->known) return QStringLiteral("设备参数状态未知");
    return e->decode(e->device)=="0" ? QStringLiteral("无待应用参数 · Flash 单独保存") : QStringLiteral("RAM 参数已变更 · 需要应用");
}
#include "Backend.moc"
