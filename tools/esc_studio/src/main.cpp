#include "Backend.h"
#include "WaveformItem.h"
#include <QGuiApplication>
#include <QFont>
#include <QFontDatabase>
#include <QFile>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QQuickWindow>
#include <QTimer>
#include <QImage>
#include <QDebug>
#include <functional>

int main(int argc,char **argv)
{
    QGuiApplication app(argc,argv);
    // The offscreen platform does not discover Windows fonts automatically.
#ifdef Q_OS_WIN
    if(QGuiApplication::platformName()=="offscreen") {
        QFontDatabase::addApplicationFont("C:/Windows/Fonts/msyh.ttc");
        QFontDatabase::addApplicationFont("C:/Windows/Fonts/segoeui.ttf");
        QFontDatabase::addApplicationFont("C:/Windows/Fonts/consola.ttf");
    }
#endif
    app.setApplicationName("J_ESC Studio"); app.setOrganizationName("J_ESC");
    app.setFont(QFont(QStringLiteral("Microsoft YaHei UI"),10));
    QQuickStyle::setStyle("Basic");
    registerScopeTypes();
    Backend backend;
    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("studio",&backend);
    QObject::connect(&engine,&QQmlApplicationEngine::objectCreationFailed,&app,[]{ QCoreApplication::exit(1); },Qt::QueuedConnection);
    engine.loadFromModule("JescStudio","Main");
    if(engine.rootObjects().isEmpty()) return 1;
    if(app.arguments().contains("--demo")) backend.openDemo();
    const int page=app.arguments().indexOf("--page");
    if(page>=0 && page+1<app.arguments().size()) engine.rootObjects().first()->setProperty("selectedPage",app.arguments()[page+1].toInt());
    const int replay=app.arguments().indexOf("--replay");
    const bool replayFile=replay>=0 && replay+1<app.arguments().size();
    if(replayFile) {
        engine.rootObjects().first()->setProperty("selectedPage",8);
        backend.scope()->openRecording(QUrl::fromLocalFile(app.arguments()[replay+1]));
    }
    const bool scopeDemo=app.arguments().contains("--scope-demo") && app.arguments().contains("--demo");
    if(scopeDemo) {
        engine.rootObjects().first()->setProperty("selectedPage",8);
        auto timer=new QTimer(&app);timer->setInterval(30);
        QObject::connect(timer,&QTimer::timeout,&app,[&,timer] {
            if(backend.connected() && !backend.busy()){timer->stop();backend.scope()->start();}
        });timer->start();
    }
    const int screenshot=app.arguments().indexOf("--screenshot");
    if(app.arguments().contains("--smoke-test")) {
        QTimer::singleShot(scopeDemo?2200:900,&app,[&] {
            auto *window=qobject_cast<QQuickWindow *>(engine.rootObjects().first());
            if(!window) { app.exit(2); return; }
            std::function<WaveformItem *(QQuickItem *)> findPlot=[&](QQuickItem *item)->WaveformItem * {
                if(auto plot=qobject_cast<WaveformItem *>(item))return plot;
                for(auto child:item->childItems())if(auto plot=findPlot(child))return plot;
                return nullptr;
            };
            auto plot=findPlot(window->contentItem());
            if(screenshot>=0 && screenshot+1<app.arguments().size()) {
                QImage image=window->grabWindow();
                if(image.isNull() || !image.save(app.arguments()[screenshot+1])) { app.exit(3); return; }
            }
            QFile result(QCoreApplication::applicationDirPath()+"/smoke-result.txt");
            if(result.open(QIODevice::WriteOnly)) {
                result.write(QStringLiteral("QML loaded; entries=%1; connected=%2; page=%3; renderer=%4; displayPoints=%5; error=%6\n")
                    .arg(backend.dictionary()->rowCount()).arg(backend.connected())
                    .arg(window->property("selectedPage").toInt()).arg(plot?plot->renderer():"none").arg(plot?plot->displayPoints():0)
                    .arg(backend.scope()->info().value("error").toString()).toUtf8());
            }
            if(scopeDemo && (!plot || plot->displayPoints()==0 || !backend.scope()->info().value("error").toString().isEmpty())){app.exit(4);return;}
            if(replayFile && (backend.scope()->fileBusy() || backend.scope()->state()!=QStringLiteral("文件回放"))){app.exit(5);return;}
            app.exit(0);
        });
    }
    return app.exec();
}
