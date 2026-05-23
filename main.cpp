#include "mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    QSurfaceFormat defaultFormat;
    defaultFormat.setRenderableType(QSurfaceFormat::OpenGL);
    defaultFormat.setProfile(QSurfaceFormat::CompatibilityProfile);
    defaultFormat.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    defaultFormat.setSwapInterval(1);
    defaultFormat.setDepthBufferSize(24);
    defaultFormat.setStencilBufferSize(8);
    defaultFormat.setSamples(8);
    QSurfaceFormat::setDefaultFormat(defaultFormat);

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
