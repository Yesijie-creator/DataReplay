#ifndef ACCELERATIONVISUALIZATIONWIDGET_H
#define ACCELERATIONVISUALIZATIONWIDGET_H

#include "datafileparser.h"

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QSize>

class AccelerationVisualizationWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit AccelerationVisualizationWidget(QWidget *parent = nullptr);
    ~AccelerationVisualizationWidget() override;

    void setSample(const ReplaySample &sample);
    void clearSample();
    void setVisualRangeG(double rangeG);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void showEvent(QShowEvent *event) override;

private:
    void updateStaticGeometry();
    void drawOverlay(const QMatrix4x4 &mvp) const;
    QSize framebufferSizeInPixels(int logicalWidth, int logicalHeight) const;
    void requestRender();

    QOpenGLShaderProgram m_program;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_staticBuffer;
    QOpenGLBuffer m_dynamicBuffer;

    ReplaySample m_sample;
    QPoint m_lastMousePos;
    bool m_hasSample = false;
    bool m_pendingRender = false;
    int m_staticVertexCount = 0;
    float m_viewYawDeg = 28.0f;
    float m_viewPitchDeg = -18.0f;
    float m_cameraDistance = 5.0f;
    float m_visualRangeG = 0.6f;
};

#endif // ACCELERATIONVISUALIZATIONWIDGET_H
