#include "DemoScope.h"
#include <QSet>
#include <cmath>
#include <cstring>

namespace {
QByteArray packetBytes(const OD_WireFrame &frame)
{
    uint8_t bytes[OD_WIRE_ENCODED_SIZE];auto size=od_wire_encode(&frame,bytes,sizeof(bytes));
    return QByteArray(reinterpret_cast<char *>(bytes),qsizetype(size));
}
}
DemoScope::DemoScope(QObject *parent):QObject(parent),m_timer(new QTimer(this))
{
    m_timer->setInterval(5);
    connect(m_timer,&QTimer::timeout,this,[this] {
        produce();
        for(int budget=0;budget<8;++budget) {
            if(m_benchmark) {
                OD_WireFrame frame{};frame.type=OD_MSG_BENCHMARK_DATA;frame.session=0xD3000001;
                frame.sequence=quint32(m_benchSequence++);frame.length=1024;
                for(int i=0;i<1024;++i)frame.payload[i]=quint8(i);
                --m_benchmark;emit packet(packetBytes(frame));
            } else if(!(m_mode==2 && m_state==1) && !m_ready.isEmpty()) emit packet(m_ready.dequeue());
            else break;
        }
    });
}
void DemoScope::reset()
{
    shutdown();m_config=0;m_segment=0;m_captured=0;m_overflows=0;m_stride=0;m_channels.clear();
    m_clock.start();m_timer->start();
}
void DemoScope::shutdown()
{m_timer->stop();m_ready.clear();m_partial.clear();m_partialCount=0;m_state=0;m_benchmark=0;}
void DemoScope::status(OD_WireFrame *out)
{
    out->length=29;uint8_t *p=out->payload+1;
    p[0]=quint8(m_state);p[1]=quint8(m_mode);p[2]=quint8(m_channels.size());p[3]=0;
    od_wire_put32(p+4,m_config);od_wire_put32(p+8,m_segment);od_wire_put32(p+12,m_captured);
    od_wire_put32(p+16,m_overflows);od_wire_put32(p+20,m_divisor);
    od_wire_put16(p+24,quint16(m_stride));od_wire_put16(p+26,quint16(m_perBlock));
}
bool DemoScope::handle(const OD_WireFrame &in,OD_WireFrame *out,const QHash<quint32,Entry> &dictionary)
{
    if(in.type==OD_MSG_BENCHMARK) {
        if(in.length!=2){out->payload[0]=5;return true;}
        if(m_state==1 || !m_ready.isEmpty() || m_benchmark){out->payload[0]=0x0C;return true;}
        int count=od_wire_u16(in.payload);
        if(count<1 || count>512){out->payload[0]=11;return true;}
        m_benchmark=count;m_benchSequence=0;return true;
    }
    if(in.type<OD_MSG_SCOPE_CONFIG || in.type>OD_MSG_SCOPE_STATUS)return false;
    if(in.type==OD_MSG_SCOPE_CONFIG) {
        if(in.length<6){out->payload[0]=5;return true;}
        if(m_state==1 || !m_ready.isEmpty() || m_benchmark){out->payload[0]=0x0C;return true;}
        int mode=in.payload[0],count=in.payload[1];quint32 divisor=od_wire_u32(in.payload+2);
        if(in.length!=6+count*5 || count<1 || count>16 || mode<1 || mode>2 || divisor<1 || divisor>2) {
            out->payload[0]=11;return true;
        }
        QVector<Entry> channels;QSet<quint32> seen;int stride=0;
        for(int i=0;i<count;++i) {
            auto p=in.payload+6+i*5;quint32 key=(quint32(od_wire_u16(p))<<8)|p[2];
            if(!dictionary.contains(key)){out->payload[0]=2;return true;}
            const auto &e=dictionary[key];
            if(e.type!=od_wire_u16(p+3) || e.size>4 || e.access!="ro" || e.index<0x2100 || e.index>0x2109 ||
                !e.subindex || seen.contains(key)){out->payload[0]=6;return true;}
            channels.append(e);seen.insert(key);stride+=e.size;
        }
        m_channels=channels;m_stride=stride;m_perBlock=1024/stride;m_divisor=divisor;m_mode=mode;m_state=0;++m_config;
        status(out);return true;
    }
    if(in.length!=0){out->payload[0]=5;return true;}
    if(in.type==OD_MSG_SCOPE_START) {
        if(m_state==1 || !m_ready.isEmpty() || m_benchmark){out->payload[0]=0x0C;return true;}
        if(m_channels.isEmpty()){out->payload[0]=10;return true;}
        m_captured=0;m_sequence=0;m_partial.clear();m_partialCount=0;++m_segment;
        m_firstTick=quint64(m_clock.nsecsElapsed()/50000);m_state=1;status(out);return true;
    }
    if(in.type==OD_MSG_SCOPE_STOP) {
        produce();if(m_partialCount)publish();m_state=0;m_benchmark=0;return true;
    }
    produce();status(out);return true;
}
void DemoScope::publish()
{
    OD_WireFrame frame{};frame.type=OD_MSG_SAMPLES;frame.session=0xD3000001;
    frame.sequence=m_sequence++;frame.length=quint16(32+m_partial.size());
    auto p=frame.payload;od_wire_put32(p,m_config);od_wire_put32(p+4,m_segment);od_wire_put32(p+8,frame.sequence);
    od_wire_put32(p+12,20000);od_wire_put32(p+16,quint32(m_partialTick));od_wire_put32(p+20,quint32(m_partialTick>>32));
    od_wire_put32(p+24,m_divisor);od_wire_put16(p+28,quint16(m_partialCount));od_wire_put16(p+30,quint16(m_stride));
    std::memcpy(p+32,m_partial.constData(),size_t(m_partial.size()));m_ready.enqueue(packetBytes(frame));
    m_partial.clear();m_partialCount=0;
}
void DemoScope::produce()
{
    if(m_state!=1)return;
    quint64 tick=quint64(m_clock.nsecsElapsed()/50000);
    quint64 target=(tick-m_firstTick)/m_divisor+1;
    while(m_captured<target) {
        if(m_ready.size()>=16){m_state=3;++m_overflows;return;}
        if(!m_partialCount)m_partialTick=m_firstTick+quint64(m_captured)*m_divisor;
        double time=double(m_firstTick+quint64(m_captured)*m_divisor)/20000;
        for(int c=0;c<m_channels.size();++c) {
            const auto &e=m_channels[c];
            double value=3*std::sin(6.283185307179586*110*time+c*2.0943951024)+0.14*std::sin(6.283185307179586*2300*time);
            if(m_captured%2000==17)value+=3.8;
            quint32 raw;
            if(e.type==8){float f=float(value);std::memcpy(&raw,&f,4);}
            else raw=e.type==1?quint32(value>0):quint32(int(std::abs(value)*100));
            for(int b=0;b<e.size;++b)m_partial.append(char(raw>>(8*b)));
        }
        ++m_partialCount;++m_captured;
        if(m_partialCount==m_perBlock)publish();
        if(m_mode==2 && m_captured==quint32(16*m_perBlock)){m_state=2;return;}
    }
}
