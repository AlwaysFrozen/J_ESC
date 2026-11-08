#include "Backend.h"
#include "WaveformItem.h"
#include <QtQuickTest/quicktest.h>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQuickStyle>
#include <QTemporaryDir>
#include <QUrl>
#include <QQuickWindow>
#include <QImage>
#include <QCoreApplication>
#include <QFontDatabase>
#include <QGuiApplication>

class UiSetup : public QObject {
    Q_OBJECT
    QTemporaryDir temporary;
public slots:
    bool saveWindow(QObject *object,const QString &name) {
        auto window=qobject_cast<QQuickWindow *>(object);
        return window && window->grabWindow().save(QCoreApplication::applicationDirPath()+"/"+name);
    }
    void applicationAvailable() {
#ifdef Q_OS_WIN
        if(QGuiApplication::platformName()=="offscreen") {
            QFontDatabase::addApplicationFont("C:/Windows/Fonts/msyh.ttc");
            QFontDatabase::addApplicationFont("C:/Windows/Fonts/segoeui.ttf");
            QFontDatabase::addApplicationFont("C:/Windows/Fonts/consola.ttf");
        }
#endif
        QGuiApplication::setFont(QFont(QStringLiteral("Microsoft YaHei UI"),10));
        QQuickStyle::setStyle("Basic");registerScopeTypes();
    }
    void qmlEngineAvailable(QQmlEngine *engine)
    {
        engine->rootContext()->setContextProperty("studio",new Backend(engine));
        engine->rootContext()->setContextProperty("uiHarness",this);
        engine->rootContext()->setContextProperty("recordingTestUrl",QUrl::fromLocalFile(temporary.filePath("gui.jescscope")));
        engine->rootContext()->setContextProperty("exportTestUrl",QUrl::fromLocalFile(temporary.filePath("gui.csv")));
        engine->rootContext()->setContextProperty("schemeTestUrl",QUrl::fromLocalFile(temporary.filePath("gui.jescconfig")));
    }
};
QUICK_TEST_MAIN_WITH_SETUP(studio_ui,UiSetup)
#include "test_ui.moc"
