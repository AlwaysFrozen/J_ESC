#include "WaveformItem.h"
#include "od_wire.h"
#include <QtTest>
#include <QQuickRenderControl>
#include <QQuickRenderTarget>
#include <QQuickWindow>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <rhi/qrhi.h>
#include <cmath>
#include <cstring>

class RenderTests:public QObject {
    Q_OBJECT
private slots:
    void rendersEightChannelsThroughRhi()
    {
        auto buffer=std::make_shared<ScopeBuffer>();buffer->clear(8);
        QVector<Entry> channels;
        for(int c=0;c<8;++c){Entry e;e.type=8;e.size=4;e.name=QString("CH%1").arg(c+1);channels.append(e);}
        for(int block=0;block<1250;++block) {
            QByteArray payload(1056,'\0');auto p=reinterpret_cast<uint8_t *>(payload.data());
            od_wire_put32(p,1);od_wire_put32(p+4,1);od_wire_put32(p+8,quint32(block));od_wire_put32(p+12,20000);
            od_wire_put32(p+16,quint32(block*32));od_wire_put32(p+24,1);od_wire_put16(p+28,32);od_wire_put16(p+30,32);
            for(int sample=0;sample<32;++sample)for(int c=0;c<8;++c) {
                float v=float(std::sin((block*32+sample)*0.035+c*0.6));
                if((block*32+sample)%1700==11)v+=3;
                quint32 word;std::memcpy(&word,&v,4);od_wire_put32(p+32+(sample*8+c)*4,word);
            }
            QString error;auto decoded=ScopeBlock::decode(payload,quint32(block),channels,&error);
            QVERIFY2(decoded,qPrintable(error));QVERIFY(buffer->append(decoded));
        }
        ScopeViewModel model(buffer);
        QQuickRenderControl control;
        QQuickWindow scene(&control);scene.setGeometry(0,0,1200,480);scene.setColor(QColor("#0c1724"));
        QVERIFY2(control.initialize(),"Unable to create an offscreen RHI device");
        QRhi *rhi=control.rhi();QVERIFY(rhi);
        auto color=std::unique_ptr<QRhiTexture>(rhi->newTexture(QRhiTexture::RGBA8,{1200,480},1,
            QRhiTexture::RenderTarget|QRhiTexture::UsedAsTransferSource));
        QVERIFY(color->create());
        auto depth=std::unique_ptr<QRhiRenderBuffer>(rhi->newRenderBuffer(QRhiRenderBuffer::DepthStencil,{1200,480},1));
        QVERIFY(depth->create());
        QRhiTextureRenderTargetDescription description{QRhiColorAttachment(color.get())};description.setDepthStencilBuffer(depth.get());
        auto target=std::unique_ptr<QRhiTextureRenderTarget>(rhi->newTextureRenderTarget(description));
        auto pass=std::unique_ptr<QRhiRenderPassDescriptor>(target->newCompatibleRenderPassDescriptor());
        target->setRenderPassDescriptor(pass.get());QVERIFY(target->create());
        scene.setRenderTarget(QQuickRenderTarget::fromRhiRenderTarget(target.get()));
        auto plot=new WaveformItem(scene.contentItem());plot->setSize({1200,480});plot->setSource(&model);model.resetView();
        QRhiReadbackResult readback;bool complete=false;readback.completed=[&]{complete=true;};
        QElapsedTimer elapsed;elapsed.start();
        constexpr int frames=120;
        for(int frame=0;frame<frames;++frame) {
            model.pan(-0.005);
            plot->update();control.polishItems();control.beginFrame();control.sync();control.render();
            if(frame==frames-1) {
                auto batch=rhi->nextResourceUpdateBatch();batch->readBackTexture(QRhiReadbackDescription(color.get()),&readback);
                control.commandBuffer()->resourceUpdate(batch);
            }
            control.endFrame();QCOMPARE(rhi->finish(),QRhi::FrameOpSuccess);QCoreApplication::processEvents();
        }
        double milliseconds=elapsed.nsecsElapsed()/1e6;
        QVERIFY(complete);QCOMPARE(readback.pixelSize,QSize(1200,480));
        QImage image(reinterpret_cast<const uchar *>(readback.data.constData()),1200,480,QImage::Format_RGBA8888);
        if(rhi->isYUpInFramebuffer())image=image.mirrored(false,true);
        int colored=0;
        for(int y=0;y<image.height();++y)for(int x=0;x<image.width();++x) {
            QColor pixel=image.pixelColor(x,y);
            if(pixel.green()>100 && std::abs(pixel.green()-pixel.red())>35)++colored;
        }
        QVERIFY2(colored>1000,"RHI readback did not contain colored waveform geometry");
        QVERIFY(image.save("scope-rhi.png"));
        auto driver=rhi->driverInfo();
        QJsonObject metrics{{"backend",plot->renderer()},{"adapter",QString::fromUtf8(driver.deviceName)},
            {"deviceType",int(driver.deviceType)},{"channels",8},{"sampleHz",20000},{"retainedSeconds",2},
            {"viewSeconds",model.span()},{"width",1200},{"height",480},{"frames",frames},
            {"elapsedMs",milliseconds},{"syntheticFramesPerSecond",frames*1000.0/milliseconds},
            {"displayPoints",plot->displayPoints()},{"coloredPixels",colored}};
        QFile result("render-metrics.json");QVERIFY(result.open(QIODevice::WriteOnly));result.write(QJsonDocument(metrics).toJson());
        qInfo().noquote()<<QJsonDocument(metrics).toJson(QJsonDocument::Compact);
        // Scene graph nodes must release GPU resources while this RHI still exists.
        control.invalidate();delete plot;scene.setRenderTarget(QQuickRenderTarget());
    }
};
int main(int argc,char **argv)
{
    qputenv("QT_QUICK_BACKEND","rhi");
    QGuiApplication app(argc,argv);
    QQuickWindow::setSceneGraphBackend("rhi");
#ifdef Q_OS_WIN
    QQuickWindow::setGraphicsApi(QSGRendererInterface::Direct3D11);
#endif
    RenderTests tests;return QTest::qExec(&tests,argc,argv);
}
#include "test_render.moc"
