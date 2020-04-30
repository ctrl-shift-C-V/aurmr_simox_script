#pragma once

#include <vector>

#include <QtCore/QtGlobal>
#include <QtGui/QtGui>

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SoOffscreenRenderer.h>

#include "ui_DepthOffscreenRendering.h"

class DepthOffscreenRenderingExample : public QMainWindow
{
    Q_OBJECT
public:
    DepthOffscreenRenderingExample();
    ~DepthOffscreenRenderingExample() override;
    int main();

public slots:
    void quit();
    void closeEvent(QCloseEvent* event) override;
    void camYawUpdated(double y);
protected:
    Ui::MainWindowCamera UI;
    SoSeparator* sceneSep;
    SoOffscreenRenderer* camRenderer;
    std::vector<float> camDepthBuffer;
};

