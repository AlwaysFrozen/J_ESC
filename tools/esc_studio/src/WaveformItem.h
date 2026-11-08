#pragma once
#include "ScopeViewModel.h"
#include <QQuickItem>
#include <QPointer>

class WaveformItem:public QQuickItem {
    Q_OBJECT
    Q_PROPERTY(ScopeViewModel* source READ source WRITE setSource NOTIFY sourceChanged)
    Q_PROPERTY(bool autoScale READ autoScale WRITE setAutoScale NOTIFY rangeChanged)
    Q_PROPERTY(double minimum READ minimum NOTIFY rangeChanged)
    Q_PROPERTY(double maximum READ maximum NOTIFY rangeChanged)
    Q_PROPERTY(QString renderer READ renderer NOTIFY renderingChanged)
    Q_PROPERTY(int displayPoints READ displayPoints NOTIFY renderingChanged)
    Q_PROPERTY(double prepareMs READ prepareMs NOTIFY renderingChanged)
    Q_PROPERTY(int channel READ channel WRITE setChannel NOTIFY renderingChanged)
    Q_PROPERTY(QVariantList points READ points WRITE setPoints NOTIFY renderingChanged)
public:
    explicit WaveformItem(QQuickItem *parent=nullptr);
    ScopeViewModel *source() const{return m_source;}
    void setSource(ScopeViewModel *);
    bool autoScale() const{return m_autoScale;}
    void setAutoScale(bool);
    double minimum() const{return m_minimum;}
    double maximum() const{return m_maximum;}
    QString renderer() const{return m_renderer;}
    int displayPoints() const{return m_displayPoints;}
    double prepareMs() const{return m_prepareMs;}
    int channel() const {return m_channel;}
    void setChannel(int c){m_channel=c;emit renderingChanged();update();}
    QVariantList points() const {return m_points;}
    void setPoints(const QVariantList &);
    Q_INVOKABLE void setVerticalRange(double low,double high);
signals:
    void sourceChanged();void rangeChanged();void renderingChanged();
protected:
    QSGNode *updatePaintNode(QSGNode *,UpdatePaintNodeData *) override;
private:
    QPointer<ScopeViewModel> m_source;
    std::shared_ptr<ScopeBuffer> m_buffer;
    QTimer m_tick;
    quint64 m_generation=0;
    bool m_autoScale=true;
    double m_minimum=-1,m_maximum=1,m_from=0,m_to=0.1;
    QString m_renderer=QStringLiteral("等待渲染");
    int m_displayPoints=0;
    double m_prepareMs=0;
    int m_channel=-1;
    QVariantList m_points;
    QVector<QPointF> m_spectrum;
};
void registerScopeTypes();
