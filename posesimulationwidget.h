#ifndef POSESIMULATIONWIDGET_H
#define POSESIMULATIONWIDGET_H

#include "datafileparser.h"

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QSize>
#include <QVector>
#include <QVector3D>
#include <QVector4D>

class PoseSimulationWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit PoseSimulationWidget(QWidget *parent = nullptr);
    ~PoseSimulationWidget() override;

    void setSamples(const QVector<ReplaySample> &samples, const PoseSceneGeometry &sceneGeometry);
    void setSample(const ReplaySample &sample, int index = -1);
    void clearSample();
    void setRollExaggeration(double factor);
    void setPitchExaggeration(double factor);
    void setYawExaggeration(double factor);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void showEvent(QShowEvent *event) override;

private:
    struct SceneState
    {
        QVector3D trainPosition;
        float groundY = 0.0f;
        float groundDistance = 0.0f;
        float modelMinY = 0.0f;
    };

    struct PoseIndicatorState
    {
        QVector3D worldPosition;
        qint64 playbackMs = 0;
    };

    void rebuildReplayCache();
    void updateStaticGeometry();
    void updateDynamicGridGeometry(const QVector3D &trackCenter, float groundY, float aspect);
    void rebuildRailGeometry();
    void updatePoseIndicatorTrailGeometryForCurrentIndex(const QVector3D &cameraWorldPosition);
    void drawCube(const QMatrix4x4 &projection,
                  const QMatrix4x4 &view,
                  const QMatrix4x4 &model,
                  const QVector4D &color);
    void drawSphere(const QMatrix4x4 &projection,
                    const QMatrix4x4 &view,
                    const QMatrix4x4 &model,
                    const QVector4D &color);
    void drawTriangles(const QMatrix4x4 &projection,
                       const QMatrix4x4 &view,
                       const QOpenGLBuffer &buffer,
                       int firstVertex,
                       int vertexCount,
                       const QVector4D &color,
                       const QMatrix4x4 &model);
    void drawTrailTriangles(const QMatrix4x4 &projection,
                            const QMatrix4x4 &view,
                            const QOpenGLBuffer &buffer,
                            int vertexCount,
                            const QMatrix4x4 &model);
    void drawLines(const QMatrix4x4 &projection,
                   const QMatrix4x4 &view,
                   const QOpenGLBuffer &buffer,
                   int vertexCount,
                   const QVector4D &color,
                   const QMatrix4x4 &model);
    void drawOverlay() const;
    QSize framebufferSizeInPixels(int logicalWidth, int logicalHeight) const;
    void requestRender();

    QOpenGLShaderProgram m_program;
    QOpenGLShaderProgram m_trailProgram;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_cubeBuffer;
    QOpenGLBuffer m_sphereBuffer;
    QOpenGLBuffer m_groundBuffer;
    QOpenGLBuffer m_gridBuffer;
    QOpenGLBuffer m_railBuffer;
    QOpenGLBuffer m_trailCoreBuffer;
    QOpenGLBuffer m_trailGlowBuffer;

    ReplaySample m_sample;
    QVector<ReplaySample> m_samples;
    PoseSceneGeometry m_poseSceneGeometry;
    QVector<SceneState> m_sceneStates;
    QVector<PoseIndicatorState> m_poseIndicatorStates;
    QPoint m_lastMousePos;
    bool m_hasPoseSample = false;
    bool m_pendingRender = false;
    int m_currentSampleIndex = -1;
    int m_cubeVertexCount = 0;
    int m_sphereVertexCount = 0;
    int m_groundVertexCount = 0;
    int m_gridVertexCount = 0;
    int m_railVertexCount = 0;
    int m_trailCoreVertexCount = 0;
    int m_trailGlowVertexCount = 0;
    float m_viewYawDeg = -32.0f;
    float m_viewPitchDeg = -18.0f;
    float m_cameraDistance = 10.0f;
    float m_rollExaggeration = 1.0f;
    float m_pitchExaggeration = 1.0f;
    float m_yawExaggeration = 1.0f;
};

#endif // POSESIMULATIONWIDGET_H
