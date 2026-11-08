#include "ScopeViewModel.h"
#include "ScopeRecording.h"
#include "ScopeAnalysis.h"
#include <algorithm>
#include <cmath>

namespace {
const QStringList colors={"#55dfbd","#64aeff","#f4c56e","#ee83b9","#a992f1","#f48c6c","#76dae7","#c6df7b"};
}
ScopeViewModel::ScopeViewModel(std::shared_ptr<ScopeBuffer> buffer,QObject *parent):QObject(parent),m_buffer(std::move(buffer))
{
    m_fileWorker=new QObject;m_fileWorker->moveToThread(&m_fileThread);
    connect(&m_fileThread,&QThread::finished,m_fileWorker,&QObject::deleteLater);m_fileThread.start();
    m_refresh.setInterval(16);
    connect(&m_refresh,&QTimer::timeout,this,[this] {
        auto summary=m_buffer->summary();
        if(++m_refreshTick%6==0) {
            m_info["memory"]=summary["memory"];m_info["capacity"]=summary["capacity"];m_info["evicted"]=summary["evicted"];
            emit statusChanged();if(m_hasA || m_hasB)emit cursorChanged();
        }
        if(m_follow && summary["received"].toULongLong()) {
            double end=summary["end"].toDouble();if(end!=m_end){m_end=end;emit viewChanged();}
        }
    });m_refresh.start();
}
ScopeViewModel::~ScopeViewModel(){m_fileThread.quit();m_fileThread.wait();}
void ScopeViewModel::setDictionary(const Dictionary &dictionary)
{
    if(active())return;
    m_choices.clear();m_selected.clear();
    for(int i=0;i<dictionary.rowCount();++i) {
        const Entry &e=*dictionary.entry(i);
        if(e.index>=0x2100 && e.index<=0x2109 && e.subindex && e.access=="ro" && e.size<=4)m_choices.append(e);
    }
    const QStringList defaults={"motor_basic0_phase_current_U","motor_basic0_phase_current_V","motor_basic0_phase_current_W"};
    for(const QString &name:defaults)for(const auto &e:m_choices)if(e.name==name)m_selected.append(name);
    if(m_selected.isEmpty() && !m_choices.isEmpty())m_selected.append(m_choices.front().name);
    emit choicesChanged();emit statusChanged();
}
void ScopeViewModel::setConnected(bool value)
{m_connected=value;if(!value)m_starting=false;emit statusChanged();}
void ScopeViewModel::acceptMetrics(const QVariantMap &info)
{
    if(m_replay || m_fileBusy)return;
    m_info=info;m_starting=false;
    auto names=info.value("channelNames").toStringList();if(!names.isEmpty())m_displayNames=names;
    emit statusChanged();
}
void ScopeViewModel::reject(const QString &reason)
{m_starting=false;m_info["active"]=false;m_info["error"]=reason;m_info["state"]=QStringLiteral("未启动");emit statusChanged();}
QVariantList ScopeViewModel::choices() const
{
    QVariantList result;
    for(bool selected:{true,false})for(const auto &e:m_choices)
        if(m_selected.contains(e.name)==selected && (m_search.isEmpty() || e.name.contains(m_search,Qt::CaseInsensitive) || e.description.contains(m_search,Qt::CaseInsensitive)))
            result.append(QVariantMap{{"name",e.name},{"label",e.description},{"type",e.typeName}});
    return result;
}
QVariantList ScopeViewModel::legend() const
{
    QVariantList result;const auto &names=m_displayNames.isEmpty()?m_selected:m_displayNames;
    for(int i=0;i<names.size();++i){QString unit;for(const auto &e:m_choices)if(e.name==names[i])unit=e.unit;
        result.append(QVariantMap{{"name",names[i]},{"color",colors[i%colors.size()]},{"unit",unit}});}
    return result;
}
void ScopeViewModel::setSearch(const QString &value){m_search=value;emit choicesChanged();}
void ScopeViewModel::setRate(int value){if(!active() && value>0){m_rate=value;emit statusChanged();}}
void ScopeViewModel::setMode(int value){if(!active() && value>=0 && value<=2){m_mode=value;emit statusChanged();}}
void ScopeViewModel::setSingleShot(bool value){if(!active()){m_single=value;emit statusChanged();}}
void ScopeViewModel::selectChannel(const QString &name,bool selected)
{
    if(active())return;
    bool found=false;for(const auto &e:m_choices)if(e.name==name)found=true;
    if(!found)return;
    if(selected && !m_selected.contains(name)) {
        if(m_selected.size()==16){reject(QStringLiteral("最多选择 16 个通道"));return;}
        m_selected.append(name);
    } else if(!selected)m_selected.removeAll(name);
    emit choicesChanged();
}
void ScopeViewModel::start()
{
    if(active() || fileBusy())return;
    if(!m_connected){reject(QStringLiteral("请先连接设备"));return;}
    QVector<Entry> channels;
    for(const auto &name:m_selected)for(const Entry &e:m_choices)if(e.name==name)channels.append(e);
    if(channels.isEmpty()){reject(QStringLiteral("请至少选择一个通道"));return;}
    if(m_recordEnabled && (!m_recordUrl.isLocalFile() || m_recordUrl.toLocalFile().isEmpty())){reject(QStringLiteral("请选择新的录制文件"));return;}
    m_replay=false;m_fileMessage.clear();emit recordRequested(m_recordEnabled?m_recordUrl.toLocalFile():QString());
    m_starting=true;m_follow=true;m_hasA=false;m_hasB=false;
    emit statusChanged();emit cursorChanged();emit startRequested(channels,m_rate,m_mode,m_single);
}
void ScopeViewModel::openRecording(const QUrl &source){fileJob(source,{},true);}
void ScopeViewModel::exportRecording(const QUrl &source,const QUrl &destination){fileJob(source,destination,false);}
void ScopeViewModel::seekRecording(double time)
{
    if(m_replay && std::isfinite(time))fileJob(QUrl::fromLocalFile(m_info.value("path").toString()),{},true,time);
}
void ScopeViewModel::fileJob(const QUrl &source,const QUrl &destination,bool playback,double windowEnd)
{
    if(active() || fileBusy())return;
    if(!source.isLocalFile() || (!playback && !destination.isLocalFile())){m_fileMessage=QStringLiteral("请选择本地文件");emit statusChanged();return;}
    m_fileBusy=true;m_fileMessage=playback?QStringLiteral("正在校验并载入记录…"):QStringLiteral("正在校验并导出 CSV…");emit statusChanged();
    double windowSpan=m_span;
    QMetaObject::invokeMethod(m_fileWorker,[this,source,destination,playback,windowEnd,windowSpan]{
        QVariantMap result;QString error;
        // Build a separate bounded history; failed files never replace the current plot.
        auto loaded=playback?std::make_shared<ScopeBuffer>():std::shared_ptr<ScopeBuffer>();
        bool ok=ScopeRecording::read(source.toLocalFile(),loaded,playback?QString():destination.toLocalFile(),&result,&error,windowEnd,windowSpan);
        QMetaObject::invokeMethod(this,[this,loaded,result,error,ok,playback,windowEnd]{
            m_fileBusy=false;
            if(ok && playback) {
                m_buffer->clear(result.value("channelNames").toStringList().size());
                for(const auto &block:loaded->snapshot())m_buffer->append(block);
                m_info=result;m_info["state"]=QStringLiteral("文件回放");m_info["active"]=false;m_replay=true;
                m_displayNames=result.value("channelNames").toStringList();m_rate=result.value("rate").toInt();
                m_hasA=false;m_hasB=false;
                if(windowEnd>=0){m_end=windowEnd;m_follow=false;emit viewChanged();}else{m_follow=true;resetView();}
                emit cursorChanged();
            }
            m_fileMessage=ok?(playback?QStringLiteral("记录校验通过；可使用回放位置载入任意时间窗口，查看原始样本与游标。"):
                QStringLiteral("CSV 导出完成，共 %1 个原始采样点。").arg(result.value("received").toULongLong())):QStringLiteral("文件处理失败：%1").arg(error);
            emit statusChanged();
        },Qt::QueuedConnection);
    },Qt::QueuedConnection);
}
void ScopeViewModel::stop(){if(active())emit stopRequested();}
void ScopeViewModel::analyze(int channel)
{
    if(m_analysisBusy || m_fileBusy)return;
    auto blocks=m_buffer->snapshot();double from=viewStart(),to=viewEnd();m_analysisBusy=true;emit analysisChanged();
    QMetaObject::invokeMethod(m_fileWorker,[this,blocks,channel,from,to]{
        auto result=ScopeAnalysis::analyze(blocks,channel,from,to);
        QMetaObject::invokeMethod(this,[this,result]{m_analysis=result;m_analysisBusy=false;emit analysisChanged();},Qt::QueuedConnection);
    },Qt::QueuedConnection);
}
void ScopeViewModel::clampEnd()
{
    auto summary=m_buffer->summary();if(summary["received"].toULongLong())
        m_end=std::clamp(m_end,summary["start"].toDouble()+1.0/std::max(1,m_rate),summary["end"].toDouble()+1.0/std::max(1,m_rate));
}
void ScopeViewModel::setFollowing(bool value)
{m_follow=value;if(value){auto s=m_buffer->summary();if(s["received"].toULongLong())m_end=s["end"].toDouble();}emit viewChanged();}
void ScopeViewModel::zoom(double factor,double anchor)
{
    if(!std::isfinite(factor) || factor<=0)return;
    anchor=std::clamp(anchor,0.0,1.0);
    double time=viewStart()+anchor*m_span,newSpan=std::clamp(m_span*factor,2.0/std::max(10000,m_rate),300.0);
    m_follow=false;m_span=newSpan;m_end=time+(1-anchor)*m_span;clampEnd();emit viewChanged();emit cursorChanged();
}
void ScopeViewModel::pan(double fraction)
{if(!std::isfinite(fraction))return;m_follow=false;m_end+=fraction*m_span;clampEnd();emit viewChanged();emit cursorChanged();}
void ScopeViewModel::resetView()
{m_span=0.1;setFollowing(true);emit cursorChanged();}
void ScopeViewModel::setCursor(int which,double fraction)
{
    if(!std::isfinite(fraction))return;
    double time=viewStart()+std::clamp(fraction,0.0,1.0)*m_span;
    m_follow=false;emit viewChanged();
    auto point=m_buffer->inspect(time);if(point["valid"].toBool())time=point["time"].toDouble();
    if(which==0){m_a=time;m_hasA=true;}else{m_b=time;m_hasB=true;}
    emit cursorChanged();
}
double ScopeViewModel::cursorA() const{return m_hasA?(m_a-viewStart())/m_span:-1;}
double ScopeViewModel::cursorB() const{return m_hasB?(m_b-viewStart())/m_span:-1;}
QString ScopeViewModel::cursorText() const
{
    QStringList text;
    auto describe=[&](const QString &name,double time){
        auto data=m_buffer->inspect(time);
        if(!data["valid"].toBool())return name+QStringLiteral("：采样间隙或历史已淘汰");
        QStringList values;auto raw=data["values"].toList();
        for(int i=0;i<raw.size();++i)values.append(QStringLiteral("CH%1=%2").arg(i+1).arg(raw[i].toDouble(),0,'g',6));
        return QStringLiteral("%1  %2 s  [tick %3]  %4").arg(name).arg(data["time"].toDouble(),0,'f',6).arg(data["tick"].toString(),values.join("  "));
    };
    if(m_hasA)text.append(describe("A",m_a));
    if(m_hasB)text.append(describe("B",m_b));
    if(m_hasA && m_hasB)text.append(QStringLiteral("Δt = %1 ms").arg((m_b-m_a)*1000,0,'g',7));
    return text.isEmpty()?QStringLiteral("左键放置 A 游标，右键放置 B；滚轮缩放，中键拖动时间轴。"):text.join('\n');
}
