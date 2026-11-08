#include "ProtocolSession.h"
#include <cstring>
#include <limits>

ProtocolSession::ProtocolSession(QObject *parent) : QObject(parent), m_timeout(new QTimer(this))
{
    m_timeout->setSingleShot(true);
    connect(m_timeout,&QTimer::timeout,this,[this] {
        abort(QStringLiteral("通讯超时，操作结果可能未知；已断开，请重连并读取核对，不自动重试"));
    });
}
void ProtocolSession::start(int timeout)
{
    m_queue.clear(); m_active=false; m_session=0; m_sequence=0; m_text.clear();
    od_wire_parser_init(&m_parser); m_state=Negotiating;
    m_timeout->start(timeout);
    emit outgoing(QByteArrayLiteral("!\nB 00000001\n"));
}
void ProtocolSession::abort(const QString &reason)
{
    if(m_state==Closed) return;
    m_timeout->stop(); m_state=Closed; m_active=false; m_queue.clear();
    emit failed(reason);
}
void ProtocolSession::send(quint8 kind,const QByteArray &payload)
{
    if(payload.size()>OD_WIRE_MAX_PAYLOAD || m_sequence==std::numeric_limits<quint32>::max()) {
        abort(QStringLiteral("请求长度或序号超限，请重连")); return;
    }
    OD_WireFrame frame{}; frame.type=kind; frame.session=m_session;
    frame.sequence=++m_sequence; frame.length=quint16(payload.size());
    std::memcpy(frame.payload,payload.constData(),size_t(payload.size()));
    uint8_t encoded[OD_WIRE_ENCODED_SIZE];
    size_t size=od_wire_encode(&frame,encoded,sizeof(encoded));
    if(size==0) { abort(QStringLiteral("无法编码请求")); return; }
    emit outgoing(QByteArray(reinterpret_cast<const char *>(encoded),qsizetype(size)));
}
void ProtocolSession::enqueue(int token,quint8 kind,const QByteArray &payload,int timeout,bool priority)
{
    if(m_state!=Ready || m_queue.size()>=256 || payload.size()>OD_WIRE_MAX_PAYLOAD || timeout<=0) {
        emit result(token,0xFE,{}); return;
    }
    Request request{token,kind,payload,timeout};
    if(priority) m_queue.prepend(request); else m_queue.enqueue(request);
    pump();
}
void ProtocolSession::pump()
{
    if(m_state!=Ready || m_active || m_queue.isEmpty()) return;
    m_current=m_queue.dequeue(); m_active=true; m_timeout->start(m_current.timeoutMs);
    send(m_current.kind,m_current.payload);
}
void ProtocolSession::ingest(const QByteArray &bytes)
{
    for(char ch:bytes) {
        if(m_state==Closed) return;
        if(m_state==Negotiating) {
            m_text+=ch;
            if(m_text.size()>96) { abort(QStringLiteral("协商响应过长")); return; }
            if(ch=='\n') {
                QByteArray line=m_text.trimmed(); m_text.clear();
                if(line=="O 00000000 80 -") continue;
                if(line!="O 00000001 00 02") { abort(QStringLiteral("固件未接受 v2 协商")); return; }
                m_state=Capabilities; send(OD_MSG_CAPABILITIES,{});
            }
        } else {
            OD_WireFrame frame;
            if(od_wire_feed(&m_parser,quint8(ch),&frame)) receive(frame);
            // A bad packet may have been a write ACK: fail the session instead of
            // guessing execution state or silently accepting gaps in future streams.
            if(m_parser.errors!=0) { abort(QStringLiteral("二进制帧校验失败，请重连并核对状态")); return; }
        }
    }
}
void ProtocolSession::receive(const OD_WireFrame &frame)
{
    if(m_state==Capabilities) {
        if(frame.type!=(OD_MSG_CAPABILITIES|OD_MSG_REPLY) || frame.sequence!=m_sequence) return;
        if(frame.session==0 || frame.length!=19 || frame.payload[0]!=0 ||
            od_wire_u32(frame.payload+1)<10000 || od_wire_u16(frame.payload+17)>OD_WIRE_MAX_PAYLOAD) {
            abort(QStringLiteral("固件能力响应无效")); return;
        }
        m_session=frame.session; m_state=Ready; m_timeout->stop();
        emit ready(QByteArray(reinterpret_cast<const char *>(frame.payload+1),18));
        pump(); return;
    }
    if(frame.session!=m_session) return;
    if(frame.type==OD_MSG_SAMPLES || frame.type==OD_MSG_BENCHMARK_DATA) {
        emit stream(frame.type,frame.sequence,QByteArray(reinterpret_cast<const char *>(frame.payload),frame.length)); return;
    }
    if(!m_active || frame.type!=(m_current.kind|OD_MSG_REPLY) || frame.sequence!=m_sequence) return;
    if(frame.length<1 || (frame.payload[0]!=0 && frame.length!=1)) {
        abort(QStringLiteral("命令响应格式错误，结果未知")); return;
    }
    if(frame.payload[0]==0 && (m_current.kind==OD_MSG_READ || m_current.kind==OD_MSG_WRITE)) {
        int expected=m_current.kind==OD_MSG_WRITE ? 1 :
            (m_current.payload.size()>=6 ? 1+quint8(m_current.payload[5]) : -1);
        if(frame.length!=expected) { abort(QStringLiteral("标量响应长度不匹配，结果未知")); return; }
    }
    m_timeout->stop(); m_active=false;
    Request completed=m_current;
    QQueue<Request> cancelled;
    if(frame.payload[0]!=0 && completed.kind==OD_MSG_WRITE) cancelled.swap(m_queue);
    emit result(completed.token,frame.payload[0],QByteArray(reinterpret_cast<const char *>(frame.payload+1),frame.length-1));
    for(const Request &request:cancelled) emit result(request.token,0xFE,{});
    pump();
}
