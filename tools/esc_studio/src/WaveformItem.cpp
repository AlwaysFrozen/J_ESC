#include "WaveformItem.h"
#include <QSGGeometryNode>
#include <QSGFlatColorMaterial>
#include <QSGSimpleTextureNode>
#include <QSGRendererInterface>
#include <QQuickWindow>
#include <QPainter>
#include <QPainterPath>
#include <QElapsedTimer>
#include <QtQml>
#include <algorithm>
#include <cmath>

namespace {
const QColor colors[]={QColor("#55dfbd"),QColor("#64aeff"),QColor("#f4c56e"),QColor("#ee83b9"),
    QColor("#a992f1"),QColor("#f48c6c"),QColor("#76dae7"),QColor("#c6df7b")};
struct PlotNode:QSGNode {
    QSGSimpleTextureNode *texture=nullptr;
    QVector<QSGGeometryNode *> meshes;
};
QSGGeometryNode *mesh(const QColor &color)
{
    auto node=new QSGGeometryNode;
    auto geometry=new QSGGeometry(QSGGeometry::defaultAttributes_Point2D(),0);
    geometry->setDrawingMode(QSGGeometry::DrawTriangles);
    geometry->setVertexDataPattern(QSGGeometry::DynamicPattern);
    auto material=new QSGFlatColorMaterial;material->setColor(color);
    node->setGeometry(geometry);node->setFlag(QSGNode::OwnsGeometry);
    node->setMaterial(material);node->setFlag(QSGNode::OwnsMaterial);return node;
}
void line(QVector<QPointF> &out,const QPointF &a,const QPointF &b,double thickness)
{
    QPointF d=b-a;double length=std::hypot(d.x(),d.y());
    if(length<0.001)return;
    QPointF normal(-d.y()/length*thickness/2,d.x()/length*thickness/2);
    out<<a+normal<<b+normal<<a-normal<<a-normal<<b+normal<<b-normal;
}
bool clipSegment(QPointF &a,QPointF &b,double width,double height)
{
    // Bound float GPU coordinates even when a manual Y range clips a very large value.
    QPointF d=b-a;double first=0,last=1;
    const double p[]={-d.x(),d.x(),-d.y(),d.y()};
    const double q[]={a.x()+2,width+2-a.x(),a.y()+2,height+2-a.y()};
    for(int i=0;i<4;++i) {
        if(p[i]==0){if(q[i]<0)return false;continue;}
        double t=q[i]/p[i];
        if(p[i]<0)first=std::max(first,t);else last=std::min(last,t);
        if(first>last)return false;
    }
    QPointF origin=a;
    a=origin+d*first;b=origin+d*last;
    a.setX(std::clamp(a.x(),-2.0,width+2));a.setY(std::clamp(a.y(),-2.0,height+2));
    b.setX(std::clamp(b.x(),-2.0,width+2));b.setY(std::clamp(b.y(),-2.0,height+2));return true;
}
void upload(QSGGeometryNode *node,const QVector<QPointF> &vertices)
{
    QSGGeometry *geometry=node->geometry();int wanted=int(vertices.size()),capacity=geometry->vertexCount();
    if(wanted>capacity || wanted<capacity/4) {
        int next=6;while(next<wanted)next*=2;
        geometry->allocate(next);capacity=next;
    }
    auto data=geometry->vertexDataAsPoint2D();
    for(int i=0;i<wanted;++i)data[i].set(float(vertices[i].x()),float(vertices[i].y()));
    // Qt 6.8 has no separate vertex-count setter. Degenerate triangles retain
    // reusable allocation and are ignored by rasterization.
    for(int i=wanted;i<capacity;++i)data[i].set(0,0);
    node->markDirty(QSGNode::DirtyGeometry);
}
QString apiName(QSGRendererInterface::GraphicsApi api)
{
    switch(api){case QSGRendererInterface::Direct3D11:return "Direct3D 11 / RHI";
#if QT_VERSION >= QT_VERSION_CHECK(6, 6, 0)
    case QSGRendererInterface::Direct3D12:return "Direct3D 12 / RHI";
#endif
    case QSGRendererInterface::Vulkan:return "Vulkan / RHI";case QSGRendererInterface::OpenGL:return "OpenGL / RHI";
    case QSGRendererInterface::Metal:return "Metal / RHI";case QSGRendererInterface::Software:return QStringLiteral("软件渲染回退");
    default:return QStringLiteral("未识别的渲染后端");}
}
}
WaveformItem::WaveformItem(QQuickItem *parent):QQuickItem(parent)
{
    setFlag(ItemHasContents,true);setClip(true);
    m_tick.setInterval(16);connect(&m_tick,&QTimer::timeout,this,[this] {
        if(!isVisible() || !m_source || !m_buffer)return;
        auto generation=m_buffer->generation();double from=m_source->viewStart(),to=m_source->viewEnd();
        if(generation!=m_generation || from!=m_from || to!=m_to) {
            m_generation=generation;m_from=from;m_to=to;update();
        }
    });m_tick.start();
}
void WaveformItem::setSource(ScopeViewModel *source)
{
    if(m_source==source)return;
    if(m_source)disconnect(m_source,nullptr,this,nullptr);
    m_source=source;m_buffer=source?source->buffer():nullptr;
    if(source)connect(source,&ScopeViewModel::viewChanged,this,[this]{m_from=m_source->viewStart();m_to=m_source->viewEnd();update();});
    emit sourceChanged();update();
}
void WaveformItem::setAutoScale(bool enabled){if(m_autoScale==enabled)return;m_autoScale=enabled;emit rangeChanged();update();}
void WaveformItem::setPoints(const QVariantList &points)
{
    m_points=points;m_spectrum.clear();for(const auto &p:points)m_spectrum.append(p.toPointF());
    m_from=0;m_to=m_spectrum.isEmpty()?1:std::max(1.0,m_spectrum.back().x());emit renderingChanged();update();
}
void WaveformItem::setVerticalRange(double low,double high)
{if(std::isfinite(low) && std::isfinite(high) && high>low){m_autoScale=false;m_minimum=low;m_maximum=high;emit rangeChanged();update();}}
QSGNode *WaveformItem::updatePaintNode(QSGNode *oldNode,UpdatePaintNodeData *)
{
    auto root=static_cast<PlotNode *>(oldNode);if(!root)root=new PlotNode;
    if(!window() || width()<1 || height()<1)return root;
    QElapsedTimer clock;clock.start();
    ScopeEnvelope envelope;if(m_buffer)envelope=m_buffer->envelope(m_from,m_to,int(width()));
    if(!m_spectrum.isEmpty())envelope.curves={m_spectrum};
    if(m_channel>=0)envelope.curves=m_channel<envelope.curves.size()?QVector<QVector<QPointF>>{envelope.curves[m_channel]}:QVector<QVector<QPointF>>{};
    if(m_channel>=0 || !m_spectrum.isEmpty()) {
        bool first=true;envelope.vertices=0;
        for(const auto &curve:envelope.curves)for(const auto &p:curve)if(std::isfinite(p.y())) {
            if(first){envelope.low=p.y();envelope.high=p.y();first=false;}else{envelope.low=std::min(envelope.low,p.y());envelope.high=std::max(envelope.high,p.y());}++envelope.vertices;
        }
    }
    double low=m_minimum,high=m_maximum;
    if(m_autoScale) {
        low=envelope.low;high=envelope.high;
        double padding=std::max((high-low)*0.08,std::max(0.01,std::abs(high)*0.01));
        low-=padding;high+=padding;
    }
    auto point=[&](const QPointF &p){return QPointF((p.x()-m_from)/std::max(1e-12,m_to-m_from)*width(),
        (high-p.y())/std::max(1e-12,high-low)*height());};
    auto api=window()->rendererInterface()->graphicsApi();
    if(api==QSGRendererInterface::Software) {
        if(!root->texture){root->texture=new QSGSimpleTextureNode;root->texture->setOwnsTexture(true);root->appendChildNode(root->texture);}
        QImage image(QSize(qCeil(width()),qCeil(height())),QImage::Format_ARGB32_Premultiplied);image.fill(Qt::transparent);
        QPainter painter(&image);painter.setRenderHint(QPainter::Antialiasing);
        painter.setPen(QPen(QColor("#24394d"),1));
        for(int x=1;x<10;++x)painter.drawLine(QPointF(x*width()/10,0),QPointF(x*width()/10,height()));
        for(int y=1;y<8;++y)painter.drawLine(QPointF(0,y*height()/8),QPointF(width(),y*height()/8));
        for(int c=0;c<envelope.curves.size();++c) {
            QPainterPath path;bool next=true;QPointF previous;
            for(const auto &p:envelope.curves[c]) {
                if(!std::isfinite(p.x())){next=true;continue;}
                QPointF current=point(p);
                if(!next){QPointF a=previous,b=current;if(clipSegment(a,b,width(),height())){path.moveTo(a);path.lineTo(b);}}
                previous=current;next=false;
            }
            painter.setPen(QPen(colors[(m_channel>=0?m_channel:c)%8],1.4));painter.drawPath(path);
        }
        painter.end();
        root->texture->setTexture(window()->createTextureFromImage(image));root->texture->setRect(boundingRect());
    } else {
        while(root->meshes.size()<envelope.curves.size()+1) {
            int at=root->meshes.size();auto node=mesh(at?colors[(m_channel>=0?m_channel:at-1)%8]:QColor("#24394d"));
            root->meshes.append(node);root->appendChildNode(node);
        }
        QVector<QPointF> grid;
        for(int x=1;x<10;++x)line(grid,{x*width()/10,0},{x*width()/10,height()},1);
        for(int y=1;y<8;++y)line(grid,{0,y*height()/8},{width(),y*height()/8},1);
        upload(root->meshes[0],grid);
        for(int c=0;c<root->meshes.size()-1;++c) {
            auto material=static_cast<QSGFlatColorMaterial *>(root->meshes[c+1]->material());
            QColor color=colors[(m_channel>=0?m_channel:c)%8];
            if(material->color()!=color){material->setColor(color);root->meshes[c+1]->markDirty(QSGNode::DirtyMaterial);}
            QVector<QPointF> vertices;
            if(c<envelope.curves.size()) {
                vertices.reserve(envelope.curves[c].size()*6);
                bool hasPrevious=false;QPointF previous;
                for(const auto &p:envelope.curves[c]) {
                    if(!std::isfinite(p.x())){hasPrevious=false;continue;}
                    QPointF current=point(p);
                    if(hasPrevious){QPointF a=previous,b=current;if(clipSegment(a,b,width(),height()))line(vertices,a,b,1.4);}
                    previous=current;hasPrevious=true;
                }
            }
            upload(root->meshes[c+1],vertices);
        }
    }
    QString renderer=apiName(api);int vertices=int(envelope.vertices);double ms=clock.nsecsElapsed()/1e6;
    QMetaObject::invokeMethod(this,[this,renderer,vertices,ms,low,high] {
        bool changed=m_minimum!=low || m_maximum!=high;
        m_minimum=low;m_maximum=high;m_renderer=renderer;m_displayPoints=vertices;m_prepareMs=ms;
        if(changed)emit rangeChanged();
        emit renderingChanged();
    },Qt::QueuedConnection);
    return root;
}
void registerScopeTypes()
{
    static bool registered=false;if(registered)return;registered=true;
    qmlRegisterUncreatableType<ScopeViewModel>("Jesc.Scope",1,0,"ScopeModel","Owned by Backend");
    qmlRegisterType<WaveformItem>("Jesc.Scope",1,0,"Waveform");
}
