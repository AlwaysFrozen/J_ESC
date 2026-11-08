#include "ScopeData.h"
#include "od_wire.h"
#include <QMutexLocker>
#include <QVariantList>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
double scalar(const Entry &e,const uint8_t *p)
{
    quint32 u=0; for(int i=0;i<e.size;++i) u|=quint32(p[i])<<(8*i);
    if(e.type==8) { float f; std::memcpy(&f,&u,4); return f; }
    if(e.type==2 || e.type==3 || e.type==4) {
        if(e.size<4 && (u&(1U<<(8*e.size-1)))) u|=~((1U<<(8*e.size))-1);
        qint32 s; std::memcpy(&s,&u,4); return s;
    }
    return u;
}
const double gapValue=std::numeric_limits<double>::quiet_NaN();
}
qsizetype ScopeBlock::memory() const
{ return sizeof(ScopeBlock)+256+raw.capacity()+values.capacity()*qsizetype(sizeof(double))+peaks.capacity()*qsizetype(sizeof(Peak)); }
std::shared_ptr<ScopeBlock> ScopeBlock::decode(const QByteArray &payload,quint32 seq,const QVector<Entry> &channels,QString *error)
{
    auto fail=[&](const QString &why)->std::shared_ptr<ScopeBlock> { *error=why; return {}; };
    if(payload.size()<32 || payload.size()>OD_WIRE_MAX_PAYLOAD || channels.isEmpty() || channels.size()>16)
        return fail(QStringLiteral("采样块长度或通道数无效"));
    auto p=reinterpret_cast<const uint8_t *>(payload.constData());
    auto b=std::make_shared<ScopeBlock>();
    b->config=od_wire_u32(p); b->segment=od_wire_u32(p+4); b->sequence=od_wire_u32(p+8);
    b->hz=od_wire_u32(p+12); b->tick=quint64(od_wire_u32(p+16))|(quint64(od_wire_u32(p+20))<<32);
    b->divisor=od_wire_u32(p+24); b->count=od_wire_u16(p+28); b->stride=od_wire_u16(p+30);
    int stride=0;
    for(const Entry &e:channels) {
        if((e.size!=1 && e.size!=2 && e.size!=4) || e.type==0x11 || e.type==0x15 || e.type==0x1B)
            return fail(QStringLiteral("不支持的采样标量"));
        stride+=e.size;
    }
    if(b->count==0 || b->stride!=stride || b->sequence!=seq || b->hz==0 || b->divisor==0 ||
        int(b->count)*stride>1024 || payload.size()!=32+int(b->count)*stride ||
        b->tick>std::numeric_limits<quint64>::max()-quint64(b->count)*b->divisor)
        return fail(QStringLiteral("采样块布局或时间基准不一致"));
    b->raw=payload.mid(32); b->values.resize(qsizetype(b->count)*channels.size());
    int offset=0;
    for(int i=0;i<b->count;++i) for(int c=0;c<channels.size();++c) {
        if(channels[c].type==1 && p[32+offset]>1) return fail(QStringLiteral("采样 BOOLEAN 无效"));
        b->values[i*channels.size()+c]=scalar(channels[c],p+32+offset); offset+=channels[c].size;
    }
    b->peaks.resize(((b->count+15)/16)*channels.size());
    for(int begin=0;begin<b->count;begin+=16) for(int c=0;c<channels.size();++c) {
        auto &peak=b->peaks[(begin/16)*channels.size()+c]; peak.finite=true;
        peak.low=std::numeric_limits<double>::infinity(); peak.high=-peak.low;
        for(int i=begin;i<std::min(begin+16,int(b->count));++i) {
            double v=b->values[i*channels.size()+c];
            if(!std::isfinite(v)) { peak.finite=false; continue; }
            if(v<peak.low) {peak.low=v;peak.lowAt=quint16(i);}
            if(v>peak.high) {peak.high=v;peak.highAt=quint16(i);}
        }
    }
    return b;
}
void ScopeBuffer::clear(int channels)
{
    QMutexLocker lock(&m_mutex); m_blocks.clear(); m_channels=channels;
    m_used=0; m_received=0; m_evicted=0; ++m_generation;
}
bool ScopeBuffer::append(std::shared_ptr<const ScopeBlock> block)
{
    if(!block || block->count==0)return false;
    const qsizetype size=block->memory();
    QMutexLocker lock(&m_mutex);
    if(size>m_capacity || block->values.size()!=qsizetype(block->count)*m_channels) return false;
    while(m_used+size>m_capacity && !m_blocks.empty()) {
        m_used-=m_blocks.front()->memory(); m_blocks.pop_front(); ++m_evicted;
    }
    m_received+=block->count; m_used+=size; m_blocks.push_back(std::move(block)); ++m_generation; return true;
}
QVector<std::shared_ptr<const ScopeBlock>> ScopeBuffer::snapshot() const
{
    QMutexLocker lock(&m_mutex);
    QVector<std::shared_ptr<const ScopeBlock>> result; result.reserve(qsizetype(m_blocks.size()));
    for(const auto &b:m_blocks) result.append(b);
    return result;
}
QVariantMap ScopeBuffer::summary() const
{
    QMutexLocker lock(&m_mutex);
    return {{"received",QVariant::fromValue(m_received)},{"evicted",QVariant::fromValue(m_evicted)},
        {"memory",m_used},{"capacity",m_capacity},{"channels",m_channels},
        {"start",m_blocks.empty()?0:m_blocks.front()->start()},{"end",m_blocks.empty()?0:m_blocks.back()->end()}};
}
QVariantMap ScopeBuffer::inspect(double time) const
{
    if(!std::isfinite(time)) return {{"valid",false}};
    auto blocks=snapshot();
    for(const auto &b:blocks) if(time>=b->start()-0.5*double(b->divisor)/b->hz && time<=b->end()+0.5*double(b->divisor)/b->hz) {
        int index=std::clamp(int(std::llround((time-b->start())*b->hz/b->divisor)),0,int(b->count)-1);
        int channels=int(b->values.size()/b->count); QVariantList values;
        for(int c=0;c<channels;++c) values.append(b->values[index*channels+c]);
        quint64 tick=b->tick+quint64(index)*b->divisor;
        return {{"valid",true},{"time",double(tick)/b->hz},{"tick",QString::number(tick)},{"segment",b->segment},{"values",values}};
    }
    return {{"valid",false},{"reason",QStringLiteral("采样间隙或历史已淘汰")}};
}
ScopeEnvelope ScopeBuffer::envelope(double from,double to,int pixels) const
{
    ScopeEnvelope result;
    if(!std::isfinite(from) || !std::isfinite(to) || to<=from || pixels<2) return result;
    pixels=std::min(pixels,8192);
    auto blocks=snapshot(); if(blocks.isEmpty()) return result;
    int channels=int(blocks.front()->values.size()/blocks.front()->count);
    result.curves.resize(channels);
    double low=std::numeric_limits<double>::infinity(),high=-low;
    struct Bin { bool used=false; double lo=0,hi=0,lt=0,ht=0; };
    QVector<Bin> bins(pixels+1);
    auto bucket=[&](double t){return std::clamp(int((t-from)/(to-from)*pixels),0,pixels);};
    for(int c=0;c<channels;++c) {
        int first=pixels+1,last=-1;
        auto add=[&](double t,double value) {
            if(t<from || t>to) return;
            auto &bin=bins[bucket(t)]; int at=bucket(t); first=std::min(first,at);last=std::max(last,at);
            if(!bin.used) { bin={true,value,value,t,t}; }
            else {if(value<bin.lo){bin.lo=value;bin.lt=t;} if(value>bin.hi){bin.hi=value;bin.ht=t;}}
            low=std::min(low,value);high=std::max(high,value);
        };
        auto flush=[&] {
            auto &curve=result.curves[c];
            for(int i=first;i<=last;++i) if(bins[i].used) {
                const auto &b=bins[i];
                if(b.lt<=b.ht) {curve.append({b.lt,b.lo}); if(b.lt!=b.ht)curve.append({b.ht,b.hi});}
                else {curve.append({b.ht,b.hi});curve.append({b.lt,b.lo});}
                bins[i].used=false;
            }
            if(!curve.isEmpty() && std::isfinite(curve.back().x())) curve.append({gapValue,gapValue});
            first=pixels+1;last=-1;
        };
        bool previous=false;quint32 segment=0;quint64 next=0;
        for(const auto &b:blocks) {
            if(b->end()<from || b->start()>to) continue;
            if(previous && (b->segment!=segment || b->tick!=next)) flush();
            previous=true;segment=b->segment;next=b->tick+quint64(b->count)*b->divisor;
            if(c==0)result.sourceSamples+=b->count;
            for(int i=0;i<b->count;) {
                double t=b->start()+double(i)*b->divisor/b->hz;
                const auto &peak=b->peaks[(i/16)*channels+c];
                int end=std::min(i+15,int(b->count)-1);
                double tail=b->start()+double(end)*b->divisor/b->hz;
                if(i%16==0 && peak.finite && t>=from && tail<=to && bucket(t)==bucket(tail)) {
                    add(b->start()+double(peak.lowAt)*b->divisor/b->hz,peak.low);
                    add(b->start()+double(peak.highAt)*b->divisor/b->hz,peak.high);
                    i=end+1;
                } else {
                    double value=b->values[i*channels+c];
                    if(std::isfinite(value)) add(t,value); else flush();
                    ++i;
                }
            }
        }
        flush();result.vertices+=result.curves[c].size();
    }
    if(std::isfinite(low)) { result.low=low;result.high=high; }
    return result;
}
