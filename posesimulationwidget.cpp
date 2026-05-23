#include "posesimulationwidget.h"

#include <algorithm>
#include <cstddef>
#include <limits>

#include <QMatrix4x4>
#include <QMouseEvent>
#include <QPainter>
#include <QShowEvent>
#include <QVector4D>
#include <QWheelEvent>
#include <QtMath>

namespace
{
struct VertexData
{
    QVector3D position;
};

struct TrailVertex
{
    QVector3D position;
    QVector4D color;
};

static constexpr double kSpeedAlertThresholdGs = 3.0;
static constexpr float kGroundGridCellSize = 5.0f;
static constexpr float kTrainLength = 2.0f;
static constexpr float kTrainWidth = kTrainLength * (1.55f / 3.05f);
static constexpr float kTrainHeight = kTrainLength * (0.60f / 3.05f);
static constexpr float kGroundNearThreshold = 0.05f * kGroundGridCellSize;
static constexpr float kGroundFarThreshold = 5.0f * kGroundGridCellSize;
static constexpr float kGroundSinkStep = kGroundGridCellSize;
static constexpr float kGroundRiseStep = 4.0f * kGroundGridCellSize;
static constexpr float kGroundSafetyGap = 0.02f;
static constexpr float kGroundPlaneAlpha = 0.24f;
static constexpr float kTrainBaseCenterY = kGroundNearThreshold + kTrainHeight * 0.5f + 0.10f;
static constexpr float kFrontMarkerOffsetZ = kTrainLength * (1.57f / 3.05f);
static constexpr float kFrontMarkerWidth = kTrainLength * (0.62f / 3.05f);
static constexpr float kFrontMarkerHeight = kTrainLength * (0.24f / 3.05f);
static constexpr float kFrontMarkerDepth = kTrainLength * (0.07f / 3.05f);
static constexpr float kTopMarkerOffsetY = kTrainLength * (0.40f / 3.05f);
static constexpr float kTopMarkerOffsetZ = kTrainLength * (0.95f / 3.05f);
static constexpr float kTopMarkerWidth = kTrainLength * (0.20f / 3.05f);
static constexpr float kTopMarkerHeight = kTrainLength * (0.10f / 3.05f);
static constexpr float kTopMarkerDepth = kTrainLength * (0.85f / 3.05f);
static constexpr float kRailCrossSectionScale = 4.0f;
static constexpr float kRailWidth = 0.02f * kRailCrossSectionScale;
static constexpr float kRailThickness = 0.02f * kRailCrossSectionScale;
static constexpr float kRailCenterOffsetX = kTrainWidth * 0.5f - kRailWidth * 0.5f;
static constexpr float kPoseIndicatorHeight = kTrainLength;
static constexpr float kPoseIndicatorRadius = kTrainLength * 0.16f;
static constexpr qint64 kPoseIndicatorTrailWindowMs = 9000;
static constexpr qint64 kPoseIndicatorTrailMinSampleStepMs = 35;
static constexpr float kPoseIndicatorTrailMinDistance = 0.015f;
static constexpr float kPoseIndicatorTrailCoreHeadWidth = kTrainLength * 0.22f;
static constexpr float kPoseIndicatorTrailCoreTailWidth = kTrainLength * 0.05f;
static constexpr float kPoseIndicatorTrailGlowHeadWidth = kTrainLength * 0.46f;
static constexpr float kPoseIndicatorTrailGlowTailWidth = kTrainLength * 0.10f;
static constexpr int kSphereLatitudeSegments = 12;
static constexpr int kSphereLongitudeSegments = 24;
static constexpr float kPi = 3.14159265358979323846f;

void appendLine(QVector<VertexData> *vertices, const QVector3D &start, const QVector3D &end)
{
    vertices->push_back({start});
    vertices->push_back({end});
}

void appendTriangle(QVector<VertexData> *vertices,
                    const QVector3D &a,
                    const QVector3D &b,
                    const QVector3D &c)
{
    vertices->push_back({a});
    vertices->push_back({b});
    vertices->push_back({c});
}

void appendQuad(QVector<VertexData> *vertices,
                const QVector3D &a,
                const QVector3D &b,
                const QVector3D &c,
                const QVector3D &d)
{
    appendTriangle(vertices, a, b, c);
    appendTriangle(vertices, a, c, d);
}

void appendTrailTriangle(QVector<TrailVertex> *vertices,
                         const TrailVertex &a,
                         const TrailVertex &b,
                         const TrailVertex &c)
{
    vertices->push_back(a);
    vertices->push_back(b);
    vertices->push_back(c);
}

void appendTrailQuad(QVector<TrailVertex> *vertices,
                     const TrailVertex &a,
                     const TrailVertex &b,
                     const TrailVertex &c,
                     const TrailVertex &d)
{
    appendTrailTriangle(vertices, a, b, c);
    appendTrailTriangle(vertices, a, c, d);
}

QVector<VertexData> buildUnitCubeVertices()
{
    const QVector3D lbf(-0.5f, -0.5f,  0.5f);
    const QVector3D rbf( 0.5f, -0.5f,  0.5f);
    const QVector3D rtf( 0.5f,  0.5f,  0.5f);
    const QVector3D ltf(-0.5f,  0.5f,  0.5f);
    const QVector3D lbb(-0.5f, -0.5f, -0.5f);
    const QVector3D rbb( 0.5f, -0.5f, -0.5f);
    const QVector3D rtb( 0.5f,  0.5f, -0.5f);
    const QVector3D ltb(-0.5f,  0.5f, -0.5f);

    return {
        { lbf }, { rbf }, { rtf }, { lbf }, { rtf }, { ltf },
        { rbb }, { lbb }, { ltb }, { rbb }, { ltb }, { rtb },
        { lbb }, { lbf }, { ltf }, { lbb }, { ltf }, { ltb },
        { rbf }, { rbb }, { rtb }, { rbf }, { rtb }, { rtf },
        { ltf }, { rtf }, { rtb }, { ltf }, { rtb }, { ltb },
        { lbb }, { rbb }, { rbf }, { lbb }, { rbf }, { lbf }
    };
}

QVector<VertexData> buildUnitSphereVertices()
{
    QVector<VertexData> vertices;
    vertices.reserve(kSphereLatitudeSegments * kSphereLongitudeSegments * 6);

    const auto spherePoint = [](float latitude, float longitude) {
        const float cosLatitude = qCos(latitude);
        return QVector3D(cosLatitude * qCos(longitude),
                         qSin(latitude),
                         cosLatitude * qSin(longitude));
    };

    for (int latIndex = 0; latIndex < kSphereLatitudeSegments; ++latIndex) {
        const float lat0 = -0.5f * kPi
                + static_cast<float>(latIndex) * kPi / static_cast<float>(kSphereLatitudeSegments);
        const float lat1 = -0.5f * kPi
                + static_cast<float>(latIndex + 1) * kPi / static_cast<float>(kSphereLatitudeSegments);

        for (int lonIndex = 0; lonIndex < kSphereLongitudeSegments; ++lonIndex) {
            const float lon0 = static_cast<float>(lonIndex) * (2.0f * kPi) / static_cast<float>(kSphereLongitudeSegments);
            const float lon1 = static_cast<float>(lonIndex + 1) * (2.0f * kPi) / static_cast<float>(kSphereLongitudeSegments);

            const QVector3D p00 = spherePoint(lat0, lon0);
            const QVector3D p01 = spherePoint(lat0, lon1);
            const QVector3D p10 = spherePoint(lat1, lon0);
            const QVector3D p11 = spherePoint(lat1, lon1);

            appendTriangle(&vertices, p00, p10, p11);
            appendTriangle(&vertices, p00, p11, p01);
        }
    }

    return vertices;
}

QMatrix4x4 buildPoseMatrix(float rollDeg, float pitchDeg, float yawDeg)
{
    QMatrix4x4 pose;
    pose.rotate(yawDeg, 0.0f, 1.0f, 0.0f);
    pose.rotate(pitchDeg, 1.0f, 0.0f, 0.0f);
    pose.rotate(rollDeg, 0.0f, 0.0f, 1.0f);
    return pose;
}

QMatrix4x4 buildSamplePoseMatrix(const ReplaySample &sample)
{
    return buildPoseMatrix(sample.poseValid ? static_cast<float>(sample.rollDeg) : 0.0f,
                           sample.poseValid ? static_cast<float>(sample.pitchDeg) : 0.0f,
                           sample.poseValid ? static_cast<float>(sample.yawDeg) : 0.0f);
}

QMatrix4x4 buildDisplayedPoseMatrix(const ReplaySample &sample,
                                    float rollExaggeration,
                                    float pitchExaggeration,
                                    float yawExaggeration)
{
    return buildPoseMatrix(sample.poseValid ? static_cast<float>(sample.rollDeg) * rollExaggeration : 0.0f,
                           sample.poseValid ? static_cast<float>(sample.pitchDeg) * pitchExaggeration : 0.0f,
                           sample.poseValid ? static_cast<float>(sample.yawDeg) * yawExaggeration : 0.0f);
}

QVector3D forwardDirectionForSample(const ReplaySample &sample)
{
    const QMatrix4x4 forwardPose = buildPoseMatrix(0.0f,
                                                   sample.poseValid ? static_cast<float>(sample.pitchDeg) : 0.0f,
                                                   sample.poseValid ? static_cast<float>(sample.yawDeg) : 0.0f);
    QVector3D direction = (forwardPose * QVector4D(0.0f, 0.0f, 1.0f, 0.0f)).toVector3D();
    if (direction.lengthSquared() <= 0.0f) {
        return QVector3D(0.0f, 0.0f, 1.0f);
    }
    direction.normalize();
    return direction;
}

float computeModelMinY(const QVector3D &trainPosition, const ReplaySample &sample)
{
    const QVector3D halfExtents(kTrainWidth * 0.5f,
                                kTrainHeight * 0.5f,
                                kTrainLength * 0.5f);
    const QMatrix4x4 pose = buildSamplePoseMatrix(sample);

    float minY = std::numeric_limits<float>::max();
    for (int xSign = -1; xSign <= 1; xSign += 2) {
        for (int ySign = -1; ySign <= 1; ySign += 2) {
            for (int zSign = -1; zSign <= 1; zSign += 2) {
                const QVector3D localCorner(halfExtents.x() * static_cast<float>(xSign),
                                            halfExtents.y() * static_cast<float>(ySign),
                                            halfExtents.z() * static_cast<float>(zSign));
                const QVector3D worldCorner = trainPosition
                        + (pose * QVector4D(localCorner, 0.0f)).toVector3D();
                minY = qMin(minY, worldCorner.y());
            }
        }
    }
    return minY;
}

float adjustGroundHeight(float previousGroundY, float modelMinY)
{
    float groundY = previousGroundY;
    while (modelMinY - groundY < kGroundNearThreshold) {
        groundY -= kGroundSinkStep;
    }
    while (modelMinY - groundY > kGroundFarThreshold) {
        groundY += kGroundRiseStep;
    }
    if (groundY > modelMinY - kGroundSafetyGap) {
        groundY = modelMinY - kGroundSafetyGap;
    }
    return groundY;
}

QVector<VertexData> buildVisibleGroundGridVertices(const QVector3D &trackCenter,
                                                   float groundY,
                                                   float aspect,
                                                   float cameraDistance)
{
    QVector<VertexData> vertices;
    const float safeAspect = qMax(1.0f, aspect);
    const float halfDepth = qMax(2.0f * kGroundGridCellSize,
                                 cameraDistance * 2.8f + 2.0f * kGroundGridCellSize);
    const float halfWidth = qMax(2.0f * kGroundGridCellSize,
                                 halfDepth * safeAspect + kGroundGridCellSize);
    const int xStartIndex = qFloor((trackCenter.x() - halfWidth) / kGroundGridCellSize) - 1;
    const int xEndIndex = qCeil((trackCenter.x() + halfWidth) / kGroundGridCellSize) + 1;
    const int zStartIndex = qFloor((trackCenter.z() - halfDepth) / kGroundGridCellSize) - 1;
    const int zEndIndex = qCeil((trackCenter.z() + halfDepth) / kGroundGridCellSize) + 1;
    const float minX = static_cast<float>(xStartIndex) * kGroundGridCellSize;
    const float maxX = static_cast<float>(xEndIndex) * kGroundGridCellSize;
    const float minZ = static_cast<float>(zStartIndex) * kGroundGridCellSize;
    const float maxZ = static_cast<float>(zEndIndex) * kGroundGridCellSize;

    vertices.reserve(((xEndIndex - xStartIndex + 1) + (zEndIndex - zStartIndex + 1)) * 2);
    for (int xIndex = xStartIndex; xIndex <= xEndIndex; ++xIndex) {
        const float x = static_cast<float>(xIndex) * kGroundGridCellSize;
        appendLine(&vertices, QVector3D(x, groundY, minZ), QVector3D(x, groundY, maxZ));
    }
    for (int zIndex = zStartIndex; zIndex <= zEndIndex; ++zIndex) {
        const float z = static_cast<float>(zIndex) * kGroundGridCellSize;
        appendLine(&vertices, QVector3D(minX, groundY, z), QVector3D(maxX, groundY, z));
    }
    return vertices;
}

QVector<VertexData> buildVisibleGroundPlaneVertices(const QVector3D &trackCenter,
                                                    float groundY,
                                                    float aspect,
                                                    float cameraDistance)
{
    QVector<VertexData> vertices;
    const float safeAspect = qMax(1.0f, aspect);
    const float halfDepth = qMax(2.0f * kGroundGridCellSize,
                                 cameraDistance * 2.8f + 2.0f * kGroundGridCellSize);
    const float halfWidth = qMax(2.0f * kGroundGridCellSize,
                                 halfDepth * safeAspect + kGroundGridCellSize);
    const float minX = trackCenter.x() - halfWidth;
    const float maxX = trackCenter.x() + halfWidth;
    const float minZ = trackCenter.z() - halfDepth;
    const float maxZ = trackCenter.z() + halfDepth;

    vertices.reserve(6);
    appendQuad(&vertices,
               QVector3D(minX, groundY, minZ),
               QVector3D(maxX, groundY, minZ),
               QVector3D(maxX, groundY, maxZ),
               QVector3D(minX, groundY, maxZ));
    return vertices;
}

QString formatAngle(double value)
{
    return QString::number(value, 'f', 2);
}

QColor overlayTextColorForSpeed(double speedGs)
{
    return qAbs(speedGs) > kSpeedAlertThresholdGs
            ? QColor(255, 96, 96)
            : QColor(240, 244, 250);
}

QVector3D toVector3D(double x, double y, double z)
{
    return QVector3D(static_cast<float>(x),
                     static_cast<float>(y),
                     static_cast<float>(z));
}

QVector3D poseIndicatorWorldPosition(const QVector3D &trainPosition, const QMatrix4x4 &bodyPose)
{
    return trainPosition + (bodyPose * QVector4D(0.0f, kPoseIndicatorHeight, 0.0f, 0.0f)).toVector3D();
}

float interpolateFloat(float start, float end, float t)
{
    return start + (end - start) * t;
}

QVector3D sectionCenter(const PoseSweepSection &section)
{
    return toVector3D(section.centerX, section.centerY, section.centerZ);
}

QVector3D sectionRight(const PoseSweepSection &section)
{
    return toVector3D(section.rightX, section.rightY, section.rightZ);
}

QVector3D sectionUp(const PoseSweepSection &section)
{
    return toVector3D(section.upX, section.upY, section.upZ);
}

void appendSweepSegment(QVector<VertexData> *vertices,
                        const PoseSweepSection &start,
                        const PoseSweepSection &end)
{
    const QVector3D startCenter = sectionCenter(start);
    const QVector3D endCenter = sectionCenter(end);
    const QVector3D startRight = sectionRight(start) * static_cast<float>(start.halfWidth);
    const QVector3D startUp = sectionUp(start) * static_cast<float>(start.halfHeight);
    const QVector3D endRight = sectionRight(end) * static_cast<float>(end.halfWidth);
    const QVector3D endUp = sectionUp(end) * static_cast<float>(end.halfHeight);

    const QVector3D s0 = startCenter - startRight - startUp;
    const QVector3D s1 = startCenter + startRight - startUp;
    const QVector3D s2 = startCenter + startRight + startUp;
    const QVector3D s3 = startCenter - startRight + startUp;
    const QVector3D e0 = endCenter - endRight - endUp;
    const QVector3D e1 = endCenter + endRight - endUp;
    const QVector3D e2 = endCenter + endRight + endUp;
    const QVector3D e3 = endCenter - endRight + endUp;

    appendQuad(vertices, s0, s1, e1, e0);
    appendQuad(vertices, s1, s2, e2, e1);
    appendQuad(vertices, s2, s3, e3, e2);
    appendQuad(vertices, s3, s0, e0, e3);
    appendQuad(vertices, s3, s2, s1, s0);
    appendQuad(vertices, e0, e1, e2, e3);
}
}

PoseSimulationWidget::PoseSimulationWidget(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_cubeBuffer(QOpenGLBuffer::VertexBuffer)
    , m_sphereBuffer(QOpenGLBuffer::VertexBuffer)
    , m_groundBuffer(QOpenGLBuffer::VertexBuffer)
    , m_gridBuffer(QOpenGLBuffer::VertexBuffer)
    , m_railBuffer(QOpenGLBuffer::VertexBuffer)
    , m_trailCoreBuffer(QOpenGLBuffer::VertexBuffer)
    , m_trailGlowBuffer(QOpenGLBuffer::VertexBuffer)
{
    setMinimumSize(520, 420);
    setFocusPolicy(Qt::StrongFocus);
    m_cameraDistance = 14.0f;
}

PoseSimulationWidget::~PoseSimulationWidget()
{
    if (context()) {
        makeCurrent();
        m_cubeBuffer.destroy();
        m_sphereBuffer.destroy();
        m_groundBuffer.destroy();
        m_gridBuffer.destroy();
        m_railBuffer.destroy();
        m_trailCoreBuffer.destroy();
        m_trailGlowBuffer.destroy();
        m_vao.destroy();
        doneCurrent();
    }
}

void PoseSimulationWidget::setSamples(const QVector<ReplaySample> &samples, const PoseSceneGeometry &sceneGeometry)
{
    m_samples = samples;
    m_poseSceneGeometry = sceneGeometry;
    rebuildReplayCache();
    rebuildRailGeometry();
    if (m_currentSampleIndex >= m_sceneStates.size()) {
        m_currentSampleIndex = m_sceneStates.isEmpty() ? -1 : (m_sceneStates.size() - 1);
    }
    requestRender();
}

void PoseSimulationWidget::setSample(const ReplaySample &sample, int index)
{
    m_sample = sample;
    m_hasPoseSample = sample.poseValid;
    if (index >= 0) {
        m_currentSampleIndex = qMin(index, m_sceneStates.size() - 1);
    }
    requestRender();
}

void PoseSimulationWidget::clearSample()
{
    m_sample = ReplaySample();
    m_samples.clear();
    m_poseSceneGeometry = PoseSceneGeometry();
    m_sceneStates.clear();
    m_poseIndicatorStates.clear();
    m_hasPoseSample = false;
    m_currentSampleIndex = -1;
    m_groundVertexCount = 0;
    m_gridVertexCount = 0;
    m_railVertexCount = 0;
    m_trailCoreVertexCount = 0;
    m_trailGlowVertexCount = 0;
    requestRender();
}

void PoseSimulationWidget::setRollExaggeration(double factor)
{
    m_rollExaggeration = qBound(1.0f, static_cast<float>(factor), 50.0f);
    requestRender();
}

void PoseSimulationWidget::setPitchExaggeration(double factor)
{
    m_pitchExaggeration = qBound(1.0f, static_cast<float>(factor), 50.0f);
    requestRender();
}

void PoseSimulationWidget::setYawExaggeration(double factor)
{
    m_yawExaggeration = qBound(1.0f, static_cast<float>(factor), 3.0f);
    requestRender();
}

void PoseSimulationWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.08f, 0.10f, 0.15f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Vertex,
                                           "attribute vec3 position;\n"
                                           "varying vec4 vColor;\n"
                                           "uniform mat4 mvp;\n"
                                           "uniform vec4 color;\n"
                                           "uniform float pointSize;\n"
                                           "void main()\n"
                                           "{\n"
                                           "    vColor = color;\n"
                                           "    gl_PointSize = pointSize;\n"
                                           "    gl_Position = mvp * vec4(position, 1.0);\n"
                                           "}\n")) {
        return;
    }

    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Fragment,
                                           "varying vec4 vColor;\n"
                                           "void main()\n"
                                           "{\n"
                                           "    gl_FragColor = vColor;\n"
                                           "}\n")) {
        return;
    }

    if (!m_program.link()) {
        return;
    }

    if (!m_trailProgram.addShaderFromSourceCode(QOpenGLShader::Vertex,
                                                "attribute vec3 position;\n"
                                                "attribute vec4 color;\n"
                                                "varying vec4 vColor;\n"
                                                "uniform mat4 mvp;\n"
                                                "void main()\n"
                                                "{\n"
                                                "    vColor = color;\n"
                                                "    gl_Position = mvp * vec4(position, 1.0);\n"
                                                "}\n")) {
        return;
    }

    if (!m_trailProgram.addShaderFromSourceCode(QOpenGLShader::Fragment,
                                                "varying vec4 vColor;\n"
                                                "void main()\n"
                                                "{\n"
                                                "    gl_FragColor = vColor;\n"
                                                "}\n")) {
        return;
    }

    if (!m_trailProgram.link()) {
        return;
    }

    m_vao.create();
    m_vao.bind();

    m_cubeBuffer.create();
    m_cubeBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_sphereBuffer.create();
    m_sphereBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_groundBuffer.create();
    m_groundBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_gridBuffer.create();
    m_gridBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_railBuffer.create();
    m_railBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_trailCoreBuffer.create();
    m_trailCoreBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_trailGlowBuffer.create();
    m_trailGlowBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    updateStaticGeometry();
    rebuildRailGeometry();

    m_vao.release();
}

void PoseSimulationWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_program.isLinked()) {
        return;
    }

    // QPainter 会在前一帧修改 OpenGL 状态，这里每帧都显式恢复深度与混合配置。
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glEnable(GL_MULTISAMPLE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);

    QMatrix4x4 projection;
    const float aspect = height() > 0 ? float(width()) / float(height()) : 1.0f;
    projection.perspective(45.0f, aspect, 0.1f, 600.0f);

    const bool hasSceneState = m_currentSampleIndex >= 0 && m_currentSampleIndex < m_sceneStates.size();
    const SceneState sceneState = hasSceneState ? m_sceneStates.at(m_currentSampleIndex) : SceneState();
    const QVector3D trackCenter = hasSceneState ? sceneState.trainPosition : QVector3D(0.0f, kTrainBaseCenterY, 0.0f);
    const float groundY = hasSceneState ? sceneState.groundY : 0.0f;
    const bool hasPoseIndicatorState = m_currentSampleIndex >= 0
            && m_currentSampleIndex < m_poseIndicatorStates.size();
    const QVector3D cameraTarget = trackCenter;

    QMatrix4x4 view;
    view.translate(0.0f, 0.0f, -m_cameraDistance);
    view.rotate(m_viewPitchDeg, 1.0f, 0.0f, 0.0f);
    view.rotate(m_viewYawDeg, 0.0f, 1.0f, 0.0f);
    view.translate(-cameraTarget.x(), -cameraTarget.y(), -cameraTarget.z());
    const QVector3D cameraWorldPosition = (view.inverted() * QVector4D(0.0f, 0.0f, 0.0f, 1.0f)).toVector3D();

    updateDynamicGridGeometry(trackCenter, groundY, aspect);
    updatePoseIndicatorTrailGeometryForCurrentIndex(cameraWorldPosition);

    QMatrix4x4 identityModel;

    const QMatrix4x4 bodyPose = buildDisplayedPoseMatrix(m_sample,
                                                         m_rollExaggeration,
                                                         m_pitchExaggeration,
                                                         m_yawExaggeration);
    const QVector3D poseIndicatorPosition = hasPoseIndicatorState
            ? m_poseIndicatorStates.at(m_currentSampleIndex).worldPosition
            : poseIndicatorWorldPosition(trackCenter, buildSamplePoseMatrix(m_sample));

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    QMatrix4x4 bodyModel;
    bodyModel.translate(trackCenter);
    bodyModel *= bodyPose;
    bodyModel.scale(kTrainWidth, kTrainHeight, kTrainLength);
    drawCube(projection, view, bodyModel, QVector4D(0.30f, 0.58f, 0.92f, 1.0f));

    QMatrix4x4 leftRailContactModel;
    leftRailContactModel.translate(trackCenter);
    leftRailContactModel *= bodyPose;
    leftRailContactModel.translate(-kRailCenterOffsetX, -kTrainHeight * 0.5f - kRailThickness * 0.5f, 0.0f);
    leftRailContactModel.scale(kRailWidth, kRailThickness, kTrainLength);
    drawCube(projection, view, leftRailContactModel, QVector4D(0.60f, 0.69f, 0.77f, 0.90f));

    QMatrix4x4 rightRailContactModel;
    rightRailContactModel.translate(trackCenter);
    rightRailContactModel *= bodyPose;
    rightRailContactModel.translate(kRailCenterOffsetX, -kTrainHeight * 0.5f - kRailThickness * 0.5f, 0.0f);
    rightRailContactModel.scale(kRailWidth, kRailThickness, kTrainLength);
    drawCube(projection, view, rightRailContactModel, QVector4D(0.60f, 0.69f, 0.77f, 0.90f));

    QMatrix4x4 frontMarkerModel;
    frontMarkerModel.translate(trackCenter);
    frontMarkerModel *= bodyPose;
    frontMarkerModel.translate(0.0f, 0.0f, kFrontMarkerOffsetZ);
    frontMarkerModel.scale(kFrontMarkerWidth, kFrontMarkerHeight, kFrontMarkerDepth);
    drawCube(projection, view, frontMarkerModel, QVector4D(0.98f, 0.82f, 0.26f, 1.0f));

    QMatrix4x4 topMarkerModel;
    topMarkerModel.translate(trackCenter);
    topMarkerModel *= bodyPose;
    topMarkerModel.translate(0.0f, kTopMarkerOffsetY, kTopMarkerOffsetZ);
    topMarkerModel.scale(kTopMarkerWidth, kTopMarkerHeight, kTopMarkerDepth);
    drawCube(projection, view, topMarkerModel, QVector4D(0.84f, 0.91f, 0.98f, 1.0f));

    QMatrix4x4 sphereModel;
    sphereModel.translate(poseIndicatorPosition);
    sphereModel.scale(kPoseIndicatorRadius);
    drawSphere(projection, view, sphereModel, QVector4D(0.78f, 0.94f, 1.00f, 0.96f));

    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glDepthMask(GL_FALSE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    drawTriangles(projection, view, m_groundBuffer, 0, m_groundVertexCount, QVector4D(0.18f, 0.23f, 0.30f, kGroundPlaneAlpha), identityModel);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_BLEND);
    drawLines(projection, view, m_gridBuffer, m_gridVertexCount, QVector4D(0.31f, 0.38f, 0.47f, 1.0f), identityModel);
    glEnable(GL_BLEND);
    drawTriangles(projection, view, m_railBuffer, 0, m_railVertexCount, QVector4D(0.26f, 0.31f, 0.38f, 0.32f), identityModel);
    drawTrailTriangles(projection, view, m_trailGlowBuffer, m_trailGlowVertexCount, identityModel);
    drawTrailTriangles(projection, view, m_trailCoreBuffer, m_trailCoreVertexCount, identityModel);
    glDepthMask(GL_TRUE);

    drawOverlay();
}

void PoseSimulationWidget::resizeGL(int w, int h)
{
    const QSize framebufferSize = framebufferSizeInPixels(w, h);
    glViewport(0, 0, framebufferSize.width(), framebufferSize.height());
}

void PoseSimulationWidget::showEvent(QShowEvent *event)
{
    QOpenGLWidget::showEvent(event);
    if (m_pendingRender) {
        m_pendingRender = false;
        update();
    }
}

void PoseSimulationWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastMousePos = event->pos();
}

void PoseSimulationWidget::mouseMoveEvent(QMouseEvent *event)
{
    const QPoint delta = event->pos() - m_lastMousePos;
    m_lastMousePos = event->pos();

    if (event->buttons() & Qt::LeftButton) {
        m_viewYawDeg += delta.x() * 0.55f;
        m_viewPitchDeg += delta.y() * 0.45f;
        m_viewPitchDeg = qBound(-85.0f, m_viewPitchDeg, 85.0f);
        requestRender();
    }
}

void PoseSimulationWidget::wheelEvent(QWheelEvent *event)
{
    const QPoint angleDelta = event->angleDelta();
    if (angleDelta.y() == 0) {
        event->ignore();
        return;
    }

    m_cameraDistance -= angleDelta.y() / 240.0f;
    m_cameraDistance = qBound(4.0f, m_cameraDistance, 120.0f);
    requestRender();
    event->accept();
}

void PoseSimulationWidget::rebuildReplayCache()
{
    m_sceneStates.clear();
    m_poseIndicatorStates.clear();
    if (m_samples.isEmpty()) {
        return;
    }

    m_sceneStates.resize(m_samples.size());
    m_poseIndicatorStates.resize(m_samples.size());
    QVector3D trainPosition(0.0f, kTrainBaseCenterY, 0.0f);
    float groundY = 0.0f;

    for (int index = 0; index < m_samples.size(); ++index) {
        const ReplaySample &sample = m_samples.at(index);
        if (index > 0) {
            const qint64 deltaPlaybackMs = sample.playbackMs - m_samples.at(index - 1).playbackMs;
            if (deltaPlaybackMs > 0) {
                const float deltaSeconds = static_cast<float>(deltaPlaybackMs) / 1000.0f;
                const float travelDistance = static_cast<float>(sample.derivedSpeedZGs) * deltaSeconds;
                trainPosition += forwardDirectionForSample(sample) * travelDistance;
            }
        }

        const float modelMinY = computeModelMinY(trainPosition, sample);
        groundY = adjustGroundHeight(groundY, modelMinY);

        SceneState &sceneState = m_sceneStates[index];
        sceneState.trainPosition = trainPosition;
        sceneState.groundY = groundY;
        sceneState.modelMinY = modelMinY;
        sceneState.groundDistance = modelMinY - groundY;

        PoseIndicatorState &indicatorState = m_poseIndicatorStates[index];
        indicatorState.worldPosition = poseIndicatorWorldPosition(trainPosition, buildSamplePoseMatrix(sample));
        indicatorState.playbackMs = sample.playbackMs;
    }
}

void PoseSimulationWidget::updateStaticGeometry()
{
    const QVector<VertexData> cubeVertices = buildUnitCubeVertices();
    m_cubeVertexCount = cubeVertices.size();

    m_cubeBuffer.bind();
    m_cubeBuffer.allocate(cubeVertices.constData(), cubeVertices.size() * static_cast<int>(sizeof(VertexData)));
    m_cubeBuffer.release();

    const QVector<VertexData> sphereVertices = buildUnitSphereVertices();
    m_sphereVertexCount = sphereVertices.size();
    m_sphereBuffer.bind();
    m_sphereBuffer.allocate(sphereVertices.constData(), sphereVertices.size() * static_cast<int>(sizeof(VertexData)));
    m_sphereBuffer.release();

    m_groundVertexCount = 0;
    m_gridVertexCount = 0;
    m_railVertexCount = 0;
    m_trailCoreVertexCount = 0;
    m_trailGlowVertexCount = 0;
}

void PoseSimulationWidget::updateDynamicGridGeometry(const QVector3D &trackCenter, float groundY, float aspect)
{
    const QVector<VertexData> groundVertices = buildVisibleGroundPlaneVertices(trackCenter, groundY, aspect, m_cameraDistance);
    m_groundVertexCount = groundVertices.size();
    if (m_groundVertexCount > 0) {
        m_groundBuffer.bind();
        m_groundBuffer.allocate(groundVertices.constData(), groundVertices.size() * static_cast<int>(sizeof(VertexData)));
        m_groundBuffer.release();
    }

    const QVector<VertexData> gridVertices = buildVisibleGroundGridVertices(trackCenter, groundY, aspect, m_cameraDistance);
    m_gridVertexCount = gridVertices.size();
    if (m_gridVertexCount <= 0) {
        return;
    }

    m_gridBuffer.bind();
    m_gridBuffer.allocate(gridVertices.constData(), gridVertices.size() * static_cast<int>(sizeof(VertexData)));
    m_gridBuffer.release();
}

void PoseSimulationWidget::rebuildRailGeometry()
{
    m_railVertexCount = 0;
    if (!context() || !m_railBuffer.isCreated()) {
        return;
    }
    if (m_poseSceneGeometry.leftRailSections.size() < 2
            && m_poseSceneGeometry.rightRailSections.size() < 2) {
        return;
    }

    QVector<VertexData> vertices;
    const auto estimateVertexCount = [](const QVector<PoseSweepSection> &sections) {
        return qMax(0, sections.size() - 1) * 36;
    };
    vertices.reserve(estimateVertexCount(m_poseSceneGeometry.leftRailSections)
                     + estimateVertexCount(m_poseSceneGeometry.rightRailSections));

    const auto appendAllSections = [&](const QVector<PoseSweepSection> &sections) {
        if (sections.size() < 2) {
            return;
        }

        for (int index = 1; index < sections.size(); ++index) {
            appendSweepSegment(&vertices, sections.at(index - 1), sections.at(index));
        }
    };

    appendAllSections(m_poseSceneGeometry.leftRailSections);
    appendAllSections(m_poseSceneGeometry.rightRailSections);

    m_railVertexCount = vertices.size();
    if (m_railVertexCount <= 0) {
        return;
    }

    m_railBuffer.bind();
    m_railBuffer.allocate(vertices.constData(), vertices.size() * static_cast<int>(sizeof(VertexData)));
    m_railBuffer.release();
}

void PoseSimulationWidget::updatePoseIndicatorTrailGeometryForCurrentIndex(const QVector3D &cameraWorldPosition)
{
    m_trailCoreVertexCount = 0;
    m_trailGlowVertexCount = 0;
    if (m_currentSampleIndex < 0
            || m_currentSampleIndex >= m_poseIndicatorStates.size()
            || !m_trailCoreBuffer.isCreated()
            || !m_trailGlowBuffer.isCreated()) {
        return;
    }

    QVector<PoseIndicatorState> sampledStates;
    sampledStates.reserve(160);
    const qint64 newestPlaybackMs = m_poseIndicatorStates.at(m_currentSampleIndex).playbackMs;
    const float minDistanceSquared = kPoseIndicatorTrailMinDistance * kPoseIndicatorTrailMinDistance;

    for (int index = m_currentSampleIndex; index >= 0; --index) {
        const PoseIndicatorState &state = m_poseIndicatorStates.at(index);
        if (newestPlaybackMs - state.playbackMs > kPoseIndicatorTrailWindowMs) {
            break;
        }

        bool acceptState = sampledStates.isEmpty();
        if (!acceptState) {
            const PoseIndicatorState &newerState = sampledStates.constLast();
            const bool farEnoughInTime = (newerState.playbackMs - state.playbackMs) >= kPoseIndicatorTrailMinSampleStepMs;
            const bool farEnoughInSpace = (newerState.worldPosition - state.worldPosition).lengthSquared() >= minDistanceSquared;
            acceptState = farEnoughInTime || farEnoughInSpace;
        }

        if (acceptState) {
            sampledStates.push_back(state);
        }
    }

    if (sampledStates.size() < 2) {
        return;
    }

    std::reverse(sampledStates.begin(), sampledStates.end());

    const auto buildLayer = [&](float tailWidth,
                                float headWidth,
                                const QVector3D &baseColor,
                                float tailAlpha,
                                float headAlpha) {
        QVector<TrailVertex> vertices;
        vertices.reserve((sampledStates.size() - 1) * 6);
        QVector<TrailVertex> leftVertices;
        QVector<TrailVertex> rightVertices;
        leftVertices.reserve(sampledStates.size());
        rightVertices.reserve(sampledStates.size());

        for (int index = 0; index < sampledStates.size(); ++index) {
            const QVector3D &position = sampledStates.at(index).worldPosition;
            QVector3D tangent;
            if (index == 0) {
                tangent = sampledStates.at(index + 1).worldPosition - position;
            } else if (index == sampledStates.size() - 1) {
                tangent = position - sampledStates.at(index - 1).worldPosition;
            } else {
                tangent = sampledStates.at(index + 1).worldPosition - sampledStates.at(index - 1).worldPosition;
            }

            if (tangent.lengthSquared() <= 1.0e-8f) {
                tangent = QVector3D(0.0f, 0.0f, 1.0f);
            } else {
                tangent.normalize();
            }

            QVector3D viewDirection = cameraWorldPosition - position;
            if (viewDirection.lengthSquared() <= 1.0e-8f) {
                viewDirection = QVector3D(0.0f, 1.0f, 0.0f);
            } else {
                viewDirection.normalize();
            }

            QVector3D right = QVector3D::crossProduct(tangent, viewDirection);
            if (right.lengthSquared() <= 1.0e-8f) {
                right = QVector3D::crossProduct(tangent, QVector3D(0.0f, 1.0f, 0.0f));
            }
            if (right.lengthSquared() <= 1.0e-8f) {
                right = QVector3D::crossProduct(tangent, QVector3D(1.0f, 0.0f, 0.0f));
            }
            if (right.lengthSquared() <= 1.0e-8f) {
                right = QVector3D(1.0f, 0.0f, 0.0f);
            } else {
                right.normalize();
            }

            const float t = sampledStates.size() > 1
                    ? static_cast<float>(index) / static_cast<float>(sampledStates.size() - 1)
                    : 1.0f;
            const float eased = t * t;
            const float width = interpolateFloat(tailWidth, headWidth, eased);
            const float alpha = interpolateFloat(tailAlpha, headAlpha, eased);
            const float brightness = interpolateFloat(0.45f, 1.0f, eased);
            const QVector4D color(baseColor.x() * brightness,
                                  baseColor.y() * brightness,
                                  baseColor.z() * brightness,
                                  alpha);
            const QVector3D offset = right * (width * 0.5f);
            leftVertices.push_back({position - offset, color});
            rightVertices.push_back({position + offset, color});
        }

        for (int index = 1; index < leftVertices.size(); ++index) {
            const float segmentLengthSquared = (sampledStates.at(index).worldPosition
                                                - sampledStates.at(index - 1).worldPosition).lengthSquared();
            if (segmentLengthSquared <= 1.0e-8f) {
                continue;
            }
            appendTrailQuad(&vertices,
                            leftVertices.at(index - 1),
                            rightVertices.at(index - 1),
                            rightVertices.at(index),
                            leftVertices.at(index));
        }

        return vertices;
    };

    const QVector<TrailVertex> glowVertices = buildLayer(kPoseIndicatorTrailGlowTailWidth,
                                                         kPoseIndicatorTrailGlowHeadWidth,
                                                         QVector3D(0.20f, 0.66f, 1.00f),
                                                         0.03f,
                                                         0.18f);
    m_trailGlowVertexCount = glowVertices.size();
    if (m_trailGlowVertexCount > 0) {
        m_trailGlowBuffer.bind();
        m_trailGlowBuffer.allocate(glowVertices.constData(),
                                   glowVertices.size() * static_cast<int>(sizeof(TrailVertex)));
        m_trailGlowBuffer.release();
    }

    const QVector<TrailVertex> coreVertices = buildLayer(kPoseIndicatorTrailCoreTailWidth,
                                                         kPoseIndicatorTrailCoreHeadWidth,
                                                         QVector3D(0.74f, 0.94f, 1.00f),
                                                         0.08f,
                                                         0.62f);
    m_trailCoreVertexCount = coreVertices.size();
    if (m_trailCoreVertexCount > 0) {
        m_trailCoreBuffer.bind();
        m_trailCoreBuffer.allocate(coreVertices.constData(),
                                   coreVertices.size() * static_cast<int>(sizeof(TrailVertex)));
        m_trailCoreBuffer.release();
    }
}

void PoseSimulationWidget::drawCube(const QMatrix4x4 &projection,
                                    const QMatrix4x4 &view,
                                    const QMatrix4x4 &model,
                                    const QVector4D &color)
{
    const int positionLocation = m_program.attributeLocation("position");

    m_program.bind();
    m_program.setUniformValue("mvp", projection * view * model);
    m_program.setUniformValue("color", color);
    m_program.setUniformValue("pointSize", 1.0f);

    m_vao.bind();
    m_cubeBuffer.bind();
    m_program.enableAttributeArray(positionLocation);
    m_program.setAttributeBuffer(positionLocation, GL_FLOAT, offsetof(VertexData, position), 3, sizeof(VertexData));
    glDrawArrays(GL_TRIANGLES, 0, m_cubeVertexCount);
    m_cubeBuffer.release();
    m_vao.release();
    m_program.release();
}

void PoseSimulationWidget::drawSphere(const QMatrix4x4 &projection,
                                      const QMatrix4x4 &view,
                                      const QMatrix4x4 &model,
                                      const QVector4D &color)
{
    if (m_sphereVertexCount <= 0) {
        return;
    }

    const int positionLocation = m_program.attributeLocation("position");

    m_program.bind();
    m_program.setUniformValue("mvp", projection * view * model);
    m_program.setUniformValue("color", color);
    m_program.setUniformValue("pointSize", 1.0f);

    m_vao.bind();
    m_sphereBuffer.bind();
    m_program.enableAttributeArray(positionLocation);
    m_program.setAttributeBuffer(positionLocation, GL_FLOAT, offsetof(VertexData, position), 3, sizeof(VertexData));
    glDrawArrays(GL_TRIANGLES, 0, m_sphereVertexCount);
    m_sphereBuffer.release();
    m_vao.release();
    m_program.release();
}

void PoseSimulationWidget::drawTriangles(const QMatrix4x4 &projection,
                                         const QMatrix4x4 &view,
                                         const QOpenGLBuffer &buffer,
                                         int firstVertex,
                                         int vertexCount,
                                         const QVector4D &color,
                                         const QMatrix4x4 &model)
{
    if (vertexCount <= 0) {
        return;
    }

    const int positionLocation = m_program.attributeLocation("position");

    m_program.bind();
    m_program.setUniformValue("mvp", projection * view * model);
    m_program.setUniformValue("color", color);
    m_program.setUniformValue("pointSize", 1.0f);

    m_vao.bind();
    const_cast<QOpenGLBuffer &>(buffer).bind();
    m_program.enableAttributeArray(positionLocation);
    m_program.setAttributeBuffer(positionLocation, GL_FLOAT, offsetof(VertexData, position), 3, sizeof(VertexData));
    glDrawArrays(GL_TRIANGLES, firstVertex, vertexCount);
    const_cast<QOpenGLBuffer &>(buffer).release();
    m_vao.release();
    m_program.release();
}

void PoseSimulationWidget::drawLines(const QMatrix4x4 &projection,
                                     const QMatrix4x4 &view,
                                     const QOpenGLBuffer &buffer,
                                     int vertexCount,
                                     const QVector4D &color,
                                     const QMatrix4x4 &model)
{
    if (vertexCount <= 0) {
        return;
    }

    const int positionLocation = m_program.attributeLocation("position");

    m_program.bind();
    m_program.setUniformValue("mvp", projection * view * model);
    m_program.setUniformValue("color", color);
    m_program.setUniformValue("pointSize", 1.0f);

    m_vao.bind();
    const_cast<QOpenGLBuffer &>(buffer).bind();
    m_program.enableAttributeArray(positionLocation);
    m_program.setAttributeBuffer(positionLocation, GL_FLOAT, offsetof(VertexData, position), 3, sizeof(VertexData));
    glDrawArrays(GL_LINES, 0, vertexCount);
    const_cast<QOpenGLBuffer &>(buffer).release();
    m_vao.release();
    m_program.release();
}

void PoseSimulationWidget::drawTrailTriangles(const QMatrix4x4 &projection,
                                              const QMatrix4x4 &view,
                                              const QOpenGLBuffer &buffer,
                                              int vertexCount,
                                              const QMatrix4x4 &model)
{
    if (vertexCount <= 0 || !m_trailProgram.isLinked()) {
        return;
    }

    const int positionLocation = m_trailProgram.attributeLocation("position");
    const int colorLocation = m_trailProgram.attributeLocation("color");

    m_trailProgram.bind();
    m_trailProgram.setUniformValue("mvp", projection * view * model);

    m_vao.bind();
    const_cast<QOpenGLBuffer &>(buffer).bind();
    m_trailProgram.enableAttributeArray(positionLocation);
    m_trailProgram.enableAttributeArray(colorLocation);
    m_trailProgram.setAttributeBuffer(positionLocation, GL_FLOAT, offsetof(TrailVertex, position), 3, sizeof(TrailVertex));
    m_trailProgram.setAttributeBuffer(colorLocation, GL_FLOAT, offsetof(TrailVertex, color), 4, sizeof(TrailVertex));
    glDrawArrays(GL_TRIANGLES, 0, vertexCount);
    const_cast<QOpenGLBuffer &>(buffer).release();
    m_vao.release();
    m_trailProgram.release();
}

void PoseSimulationWidget::drawOverlay() const
{
    QPainter painter(const_cast<PoseSimulationWidget *>(this));
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    const bool hasSceneState = m_currentSampleIndex >= 0 && m_currentSampleIndex < m_sceneStates.size();
    const SceneState sceneState = hasSceneState ? m_sceneStates.at(m_currentSampleIndex) : SceneState();

    const QRect infoRect(14, 14, 420, 392);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(13, 18, 27, 190));
    painter.drawRoundedRect(infoRect, 10, 10);

    const QColor defaultTextColor(240, 244, 250);
    const bool hasMovementSample = m_sample.hasImu && m_sample.correctedImu.valid;
    painter.setPen(defaultTextColor);
    QFont titleFont = painter.font();
    titleFont.setBold(true);
    titleFont.setPointSize(titleFont.pointSize() + 2);
    painter.setFont(titleFont);
    const int contentLeft = infoRect.left() + 12;
    const int contentTop = infoRect.top() + 12;
    const int contentBottom = infoRect.bottom() - 12;
    const QFontMetrics titleMetrics(titleFont);
    int y = contentTop + titleMetrics.ascent();
    painter.drawText(contentLeft, y, QStringLiteral("姿态模拟"));

    QFont bodyFont = painter.font();
    bodyFont.setBold(false);
    bodyFont.setPointSize(qMax(9, bodyFont.pointSize() - 1));
    const QFontMetrics bodyMetrics(bodyFont);

    QFont sectionTitleFont = bodyFont;
    sectionTitleFont.setBold(true);
    sectionTitleFont.setPointSize(sectionTitleFont.pointSize() + 1);
    const QFontMetrics sectionTitleMetrics(sectionTitleFont);

    QFont motionFont = bodyFont;
    motionFont.setBold(true);
    motionFont.setPointSize(motionFont.pointSize() + 1);
    const QFontMetrics motionMetrics(motionFont);

    QStringList lines;
    lines << QStringLiteral("视角: 左键拖拽旋转, 滚轮缩放");
    if (m_hasPoseSample) {
        lines << QStringLiteral("Roll: %1 deg").arg(formatAngle(m_sample.rollDeg))
              << QStringLiteral("Pitch: %1 deg").arg(formatAngle(m_sample.pitchDeg))
              << QStringLiteral("Yaw: %1 deg").arg(formatAngle(m_sample.yawDeg));
    } else {
        lines << QStringLiteral("当前姿态无效, 显示零姿态");
    }

    if (hasSceneState) {
        lines << QStringLiteral("地面高度: %1 g*s*s").arg(QString::number(sceneState.groundY, 'f', 2))
              << QStringLiteral("当前离地: %1 g*s*s").arg(QString::number(sceneState.groundDistance, 'f', 2));
    }

    lines << QStringLiteral("模型轴: X=左右  Y=上下  Z=前后")
          << QStringLiteral("展示映射: Roll->绕Z  Pitch->绕X  Yaw->绕Y")
          << QStringLiteral("姿态参考球: 车体局部上方 %1x 车长").arg(QString::number(kPoseIndicatorHeight / kTrainLength, 'f', 1))
          << QStringLiteral("球体拖尾: 最近 %1 s 渐隐渐细").arg(QString::number(static_cast<double>(kPoseIndicatorTrailWindowMs) / 1000.0, 'f', 1))
          << QStringLiteral("Roll 左右倾斜/抖动: %1x").arg(QString::number(m_rollExaggeration, 'f', 1))
          << QStringLiteral("Pitch 俯仰: %1x").arg(QString::number(m_pitchExaggeration, 'f', 1))
          << QStringLiteral("Yaw 偏航: %1x").arg(QString::number(m_yawExaggeration, 'f', 1))
          << QStringLiteral("轨道显示: 全量轨道")
          << QStringLiteral("轨道接触块: 宽 %1 / 厚 %2 / 全长 %3")
             .arg(QString::number(kRailWidth, 'f', 2))
             .arg(QString::number(kRailThickness, 'f', 2))
             .arg(QString::number(kTrainLength, 'f', 1))
          << QStringLiteral("网格边长: %1 g*s*s  车长: %2 g*s*s")
             .arg(QString::number(kGroundGridCellSize, 'f', 0))
             .arg(QString::number(kTrainLength, 'f', 0))
          << QStringLiteral("参考物: 半透明地面 + 网格");

    const QString speedText = QStringLiteral("解算速度(修正后Z轴积分): %1 g*s")
            .arg(QString::number(hasMovementSample ? m_sample.derivedSpeedZGs : 0.0, 'f', 3));
    const QString mileageText = QStringLiteral("当前里程: %1 g*s*s")
            .arg(QString::number(hasMovementSample ? m_sample.derivedMileageZGss : 0.0, 'f', 3));

    const int motionBlockHeight = sectionTitleMetrics.height() + 8 + motionMetrics.lineSpacing() * 2;
    const int motionTop = contentBottom - motionBlockHeight;

    painter.setFont(bodyFont);
    painter.setPen(defaultTextColor);
    y += titleMetrics.descent() + 12 + bodyMetrics.ascent();
    const int bodyBottomLimit = motionTop - 16;
    for (const QString &line : qAsConst(lines)) {
        if (y > bodyBottomLimit) {
            break;
        }
        painter.drawText(contentLeft, y, line);
        y += bodyMetrics.lineSpacing();
    }

    painter.setFont(sectionTitleFont);
    painter.drawText(contentLeft,
                     motionTop + sectionTitleMetrics.ascent(),
                     QStringLiteral("运动参数"));

    painter.setFont(motionFont);
    painter.setPen(overlayTextColorForSpeed(hasMovementSample ? m_sample.derivedSpeedZGs : 0.0));
    int motionY = motionTop + sectionTitleMetrics.height() + 8 + motionMetrics.ascent();
    painter.drawText(contentLeft, motionY, speedText);
    painter.setPen(defaultTextColor);
    motionY += motionMetrics.lineSpacing();
    painter.drawText(contentLeft, motionY, mileageText);
}

QSize PoseSimulationWidget::framebufferSizeInPixels(int logicalWidth, int logicalHeight) const
{
    const qreal devicePixelRatio = devicePixelRatioF();
    return QSize(qMax(1, qRound(logicalWidth * devicePixelRatio)),
                 qMax(1, qRound(logicalHeight * devicePixelRatio)));
}

void PoseSimulationWidget::requestRender()
{
    if (isVisible()) {
        update();
    } else {
        m_pendingRender = true;
    }
}
