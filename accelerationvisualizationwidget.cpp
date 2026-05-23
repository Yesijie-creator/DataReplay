#include "accelerationvisualizationwidget.h"

#include <cstddef>

#include <QMatrix4x4>
#include <QMouseEvent>
#include <QPainter>
#include <QShowEvent>
#include <QVector3D>
#include <QWheelEvent>
#include <QtMath>

namespace
{
struct VertexData
{
    QVector3D position;
    QVector3D color;
};

static constexpr float kSphereRadius = 1.25f;
static constexpr int kLongitudeSegments = 24;
static constexpr int kLatitudeSegments = 12;
static constexpr int kDynamicVertexCount = 8;

void appendLine(QVector<VertexData> *vertices,
                const QVector3D &start,
                const QVector3D &end,
                const QVector3D &color)
{
    vertices->push_back({start, color});
    vertices->push_back({end, color});
}

QVector<VertexData> buildSphereAndAxes()
{
    QVector<VertexData> vertices;
    vertices.reserve(912);

    const QVector3D xColor(0.92f, 0.36f, 0.36f);
    const QVector3D yColor(0.35f, 0.84f, 0.46f);
    const QVector3D zColor(0.31f, 0.63f, 0.96f);
    const QVector3D sphereColor(0.52f, 0.60f, 0.74f);
    const QVector3D frontColor(0.98f, 0.82f, 0.25f);

    appendLine(&vertices, QVector3D(-kSphereRadius, 0.0f, 0.0f), QVector3D(kSphereRadius, 0.0f, 0.0f), xColor);
    appendLine(&vertices, QVector3D(0.0f, -kSphereRadius, 0.0f), QVector3D(0.0f, kSphereRadius, 0.0f), yColor);
    appendLine(&vertices, QVector3D(0.0f, 0.0f, -kSphereRadius), QVector3D(0.0f, 0.0f, kSphereRadius), zColor);

    for (int lon = 0; lon < kLongitudeSegments; ++lon) {
        const float theta1 = (2.0f * float(M_PI) * lon) / float(kLongitudeSegments);
        const float theta2 = (2.0f * float(M_PI) * (lon + 1)) / float(kLongitudeSegments);

        appendLine(&vertices,
                   QVector3D(kSphereRadius * qCos(theta1), 0.0f, kSphereRadius * qSin(theta1)),
                   QVector3D(kSphereRadius * qCos(theta2), 0.0f, kSphereRadius * qSin(theta2)),
                   sphereColor);
        appendLine(&vertices,
                   QVector3D(0.0f, kSphereRadius * qCos(theta1), kSphereRadius * qSin(theta1)),
                   QVector3D(0.0f, kSphereRadius * qCos(theta2), kSphereRadius * qSin(theta2)),
                   sphereColor);
        appendLine(&vertices,
                   QVector3D(kSphereRadius * qCos(theta1), kSphereRadius * qSin(theta1), 0.0f),
                   QVector3D(kSphereRadius * qCos(theta2), kSphereRadius * qSin(theta2), 0.0f),
                   sphereColor);
    }

    for (int lat = 1; lat < kLatitudeSegments; ++lat) {
        const float phi = -0.5f * float(M_PI) + float(M_PI) * lat / float(kLatitudeSegments);
        const float y = kSphereRadius * qSin(phi);
        const float ringRadius = kSphereRadius * qCos(phi);
        for (int lon = 0; lon < kLongitudeSegments; ++lon) {
            const float theta1 = (2.0f * float(M_PI) * lon) / float(kLongitudeSegments);
            const float theta2 = (2.0f * float(M_PI) * (lon + 1)) / float(kLongitudeSegments);
            appendLine(&vertices,
                       QVector3D(ringRadius * qCos(theta1), y, ringRadius * qSin(theta1)),
                       QVector3D(ringRadius * qCos(theta2), y, ringRadius * qSin(theta2)),
                       sphereColor);
        }
    }

    const float arrowStart = kSphereRadius * 1.22f;
    const float arrowTip = kSphereRadius * 1.72f;
    const float arrowWing = 0.18f;

    appendLine(&vertices, QVector3D(0.0f, 0.0f, arrowStart), QVector3D(0.0f, 0.0f, arrowTip), frontColor);
    appendLine(&vertices, QVector3D(0.0f, 0.0f, arrowTip), QVector3D(arrowWing, 0.10f, kSphereRadius * 1.55f), frontColor);
    appendLine(&vertices, QVector3D(0.0f, 0.0f, arrowTip), QVector3D(-arrowWing, 0.10f, kSphereRadius * 1.55f), frontColor);
    appendLine(&vertices, QVector3D(0.0f, 0.0f, arrowTip), QVector3D(0.0f, -0.20f, kSphereRadius * 1.55f), frontColor);

    appendLine(&vertices, QVector3D(0.0f, arrowStart, 0.0f), QVector3D(0.0f, arrowTip, 0.0f), yColor);
    appendLine(&vertices, QVector3D(0.0f, arrowTip, 0.0f), QVector3D(arrowWing, kSphereRadius * 1.55f, 0.0f), yColor);
    appendLine(&vertices, QVector3D(0.0f, arrowTip, 0.0f), QVector3D(-arrowWing, kSphereRadius * 1.55f, 0.0f), yColor);
    appendLine(&vertices, QVector3D(0.0f, arrowTip, 0.0f), QVector3D(0.0f, kSphereRadius * 1.55f, arrowWing), yColor);

    return vertices;
}

QVector3D toVisualVector(const ReplaySample &sample)
{
    return QVector3D(static_cast<float>(-sample.accelYG),
                     static_cast<float>(1.0 - sample.accelXG),
                     static_cast<float>(-sample.accelZG));
}

QString formatSignedValue(double value)
{
    return QString::number(value, 'f', 3);
}

QPointF projectToScreen(const QVector3D &position,
                        const QMatrix4x4 &mvp,
                        int viewportWidth,
                        int viewportHeight,
                        bool *visible = nullptr)
{
    const QVector4D clip = mvp * QVector4D(position, 1.0f);
    bool isVisible = clip.w() > 0.0f;
    if (isVisible) {
        const float invW = 1.0f / clip.w();
        const float ndcX = clip.x() * invW;
        const float ndcY = clip.y() * invW;
        const float ndcZ = clip.z() * invW;
        isVisible = ndcX >= -1.2f && ndcX <= 1.2f
                && ndcY >= -1.2f && ndcY <= 1.2f
                && ndcZ >= -1.2f && ndcZ <= 1.2f;
        if (visible) {
            *visible = isVisible;
        }
        return QPointF((ndcX * 0.5f + 0.5f) * viewportWidth,
                       (1.0f - (ndcY * 0.5f + 0.5f)) * viewportHeight);
    }

    if (visible) {
        *visible = false;
    }
    return QPointF();
}
}

AccelerationVisualizationWidget::AccelerationVisualizationWidget(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_staticBuffer(QOpenGLBuffer::VertexBuffer)
    , m_dynamicBuffer(QOpenGLBuffer::VertexBuffer)
{
    setMinimumSize(520, 420);
    setFocusPolicy(Qt::StrongFocus);
}

AccelerationVisualizationWidget::~AccelerationVisualizationWidget()
{
    if (context()) {
        makeCurrent();
        m_staticBuffer.destroy();
        m_dynamicBuffer.destroy();
        m_vao.destroy();
        doneCurrent();
    }
}

void AccelerationVisualizationWidget::setSample(const ReplaySample &sample)
{
    m_sample = sample;
    m_hasSample = sample.hasImu;
    requestRender();
}

void AccelerationVisualizationWidget::clearSample()
{
    m_sample = ReplaySample();
    m_hasSample = false;
    requestRender();
}

void AccelerationVisualizationWidget::setVisualRangeG(double rangeG)
{
    m_visualRangeG = qBound(0.1f, static_cast<float>(rangeG), 1.0f);
    requestRender();
}

void AccelerationVisualizationWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.10f, 0.13f, 0.18f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_PROGRAM_POINT_SIZE);

    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Vertex,
                                           "attribute vec3 position;\n"
                                           "attribute vec3 color;\n"
                                           "varying vec3 vColor;\n"
                                           "uniform mat4 mvp;\n"
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
                                           "varying vec3 vColor;\n"
                                           "void main()\n"
                                           "{\n"
                                           "    gl_FragColor = vec4(vColor, 1.0);\n"
                                           "}\n")) {
        return;
    }

    if (!m_program.link()) {
        return;
    }

    m_vao.create();
    m_vao.bind();

    m_staticBuffer.create();
    m_dynamicBuffer.create();
    updateStaticGeometry();
    m_dynamicBuffer.bind();
    m_dynamicBuffer.allocate(kDynamicVertexCount * static_cast<int>(sizeof(VertexData)));
    m_dynamicBuffer.release();

    m_vao.release();
}

void AccelerationVisualizationWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_program.isLinked()) {
        return;
    }

    QMatrix4x4 view;
    view.translate(0.0f, 0.0f, -m_cameraDistance);
    view.rotate(m_viewPitchDeg, 1.0f, 0.0f, 0.0f);
    view.rotate(m_viewYawDeg, 0.0f, 1.0f, 0.0f);

    QMatrix4x4 projection;
    const float aspect = height() > 0 ? float(width()) / float(height()) : 1.0f;
    projection.perspective(45.0f, aspect, 0.1f, 100.0f);

    const QMatrix4x4 mvp = projection * view;

    const int positionLocation = m_program.attributeLocation("position");
    const int colorLocation = m_program.attributeLocation("color");

    m_program.bind();
    m_program.setUniformValue("mvp", mvp);

    m_vao.bind();

    m_staticBuffer.bind();
    m_program.enableAttributeArray(positionLocation);
    m_program.setAttributeBuffer(positionLocation, GL_FLOAT, offsetof(VertexData, position), 3, sizeof(VertexData));
    m_program.enableAttributeArray(colorLocation);
    m_program.setAttributeBuffer(colorLocation, GL_FLOAT, offsetof(VertexData, color), 3, sizeof(VertexData));
    m_program.setUniformValue("pointSize", 1.0f);
    glDrawArrays(GL_LINES, 0, m_staticVertexCount);
    m_staticBuffer.release();

    if (m_hasSample) {
        const QVector3D rawVector = toVisualVector(m_sample);
        QVector3D pointPosition(0.0f, 0.0f, 0.0f);
        if (!qFuzzyIsNull(rawVector.lengthSquared())) {
            const float vectorLength = rawVector.length();
            pointPosition = rawVector / qMax(vectorLength, 0.0001f)
                    * (vectorLength / m_visualRangeG) * kSphereRadius;
        }

        QVector<VertexData> dynamicVertices;
        dynamicVertices.reserve(kDynamicVertexCount);
        dynamicVertices.push_back({QVector3D(0.0f, 0.0f, 0.0f), QVector3D(0.96f, 0.80f, 0.26f)});
        dynamicVertices.push_back({pointPosition, QVector3D(0.96f, 0.80f, 0.26f)});
        dynamicVertices.push_back({pointPosition, QVector3D(0.96f, 0.92f, 0.42f)});
        dynamicVertices.push_back({pointPosition, QVector3D(0.96f, 0.92f, 0.42f)});
        dynamicVertices.push_back({QVector3D(pointPosition.x() + 0.07f, pointPosition.y(), pointPosition.z()), QVector3D(0.96f, 0.92f, 0.42f)});
        dynamicVertices.push_back({QVector3D(pointPosition.x() - 0.07f, pointPosition.y(), pointPosition.z()), QVector3D(0.96f, 0.92f, 0.42f)});
        dynamicVertices.push_back({QVector3D(pointPosition.x(), pointPosition.y() + 0.07f, pointPosition.z()), QVector3D(0.96f, 0.92f, 0.42f)});
        dynamicVertices.push_back({QVector3D(pointPosition.x(), pointPosition.y() - 0.07f, pointPosition.z()), QVector3D(0.96f, 0.92f, 0.42f)});

        m_dynamicBuffer.bind();
        m_dynamicBuffer.write(0,
                              dynamicVertices.constData(),
                              dynamicVertices.size() * static_cast<int>(sizeof(VertexData)));
        m_program.setAttributeBuffer(positionLocation, GL_FLOAT, offsetof(VertexData, position), 3, sizeof(VertexData));
        m_program.setAttributeBuffer(colorLocation, GL_FLOAT, offsetof(VertexData, color), 3, sizeof(VertexData));
        m_program.setUniformValue("pointSize", 1.0f);
        glDrawArrays(GL_LINES, 0, 2);
        m_program.setUniformValue("pointSize", 14.0f);
        glDrawArrays(GL_POINTS, 2, 1);
        glDrawArrays(GL_LINES, 3, 4);
        m_dynamicBuffer.release();
    }

    m_vao.release();
    m_program.release();

    drawOverlay(mvp);
}

void AccelerationVisualizationWidget::resizeGL(int w, int h)
{
    const QSize framebufferSize = framebufferSizeInPixels(w, h);
    glViewport(0, 0, framebufferSize.width(), framebufferSize.height());
}

void AccelerationVisualizationWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastMousePos = event->pos();
}

void AccelerationVisualizationWidget::mouseMoveEvent(QMouseEvent *event)
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

void AccelerationVisualizationWidget::wheelEvent(QWheelEvent *event)
{
    if (event->angleDelta().y() == 0) {
        event->ignore();
        return;
    }

    m_cameraDistance -= event->angleDelta().y() / 240.0f;
    m_cameraDistance = qBound(3.2f, m_cameraDistance, 10.0f);
    requestRender();
    event->accept();
}

void AccelerationVisualizationWidget::showEvent(QShowEvent *event)
{
    QOpenGLWidget::showEvent(event);
    if (m_pendingRender) {
        m_pendingRender = false;
        update();
    }
}

void AccelerationVisualizationWidget::updateStaticGeometry()
{
    const QVector<VertexData> vertices = buildSphereAndAxes();
    m_staticVertexCount = vertices.size();

    m_staticBuffer.bind();
    m_staticBuffer.allocate(vertices.constData(), vertices.size() * static_cast<int>(sizeof(VertexData)));
    m_staticBuffer.release();
}

void AccelerationVisualizationWidget::drawOverlay(const QMatrix4x4 &mvp) const
{
    QPainter painter(const_cast<AccelerationVisualizationWidget *>(this));
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    const QRect infoRect(14, 14, 360, 286);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(14, 19, 28, 190));
    painter.drawRoundedRect(infoRect, 10, 10);

    painter.setPen(QColor(238, 243, 250));
    QFont titleFont = painter.font();
    titleFont.setBold(true);
    titleFont.setPointSize(titleFont.pointSize() + 1);
    painter.setFont(titleFont);
    painter.drawText(infoRect.adjusted(12, 10, -12, 0), QStringLiteral("体感加速度可视化（未进行姿态校正）"));

    QFont bodyFont = painter.font();
    bodyFont.setBold(false);
    bodyFont.setPointSize(qMax(9, bodyFont.pointSize() - 1));
    painter.setFont(bodyFont);

    QStringList lines;
    lines << QStringLiteral("视角: 左键拖拽旋转, 滚轮缩放")
          << QStringLiteral("说明: 当前显示未进行姿态校正")
          << QStringLiteral("传感器轴语义: +X=上方  +Z=前方")
          << QStringLiteral("3D本地轴语义: +X=左右  +Y=上下  +Z=前后")
          << QStringLiteral("语义映射: 本地X=-(传感器Y)  本地Y=-(传感器X-1G)  本地Z=-(传感器Z)")
          << QStringLiteral("参考球半径量程: %1 g").arg(QString::number(m_visualRangeG, 'f', 2));
    if (m_hasSample) {
        const QVector3D vector = toVisualVector(m_sample);
        lines << QStringLiteral("本地X(左右): %1 g").arg(formatSignedValue(vector.x()))
              << QStringLiteral("本地Y(上下): %1 g").arg(formatSignedValue(vector.y()))
              << QStringLiteral("本地Z(前后): %1 g").arg(formatSignedValue(vector.z()))
              << QStringLiteral("模长: %1 g").arg(QString::number(vector.length(), 'f', 3));
    } else {
        lines << QStringLiteral("当前无 IMU 数据");
    }

    const QRect textRect = infoRect.adjusted(12, 34, -12, -10);
    painter.drawText(textRect, Qt::AlignLeft | Qt::TextWordWrap, lines.join(QStringLiteral("\n")));

    struct DirectionLabel
    {
        QString text;
        QVector3D position;
        QColor color;
    };

    const float arrowTip = kSphereRadius * 1.72f;
    const QVector<DirectionLabel> directionLabels = {
        {QStringLiteral("前方"), QVector3D(0.0f, 0.0f, arrowTip), QColor(250, 214, 79)},
        {QStringLiteral("上方"), QVector3D(0.0f, arrowTip, 0.0f), QColor(89, 214, 117)}
    };

    QFont labelFont = bodyFont;
    labelFont.setBold(true);
    painter.setFont(labelFont);
    for (const DirectionLabel &label : directionLabels) {
        bool visible = false;
        const QPointF anchor = projectToScreen(label.position, mvp, width(), height(), &visible);
        if (!visible) {
            continue;
        }

        const QRectF labelRect(anchor.x() + 8.0, anchor.y() - 12.0, 42.0, 24.0);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(16, 22, 32, 212));
        painter.drawRoundedRect(labelRect, 6.0, 6.0);
        painter.setPen(label.color);
        painter.drawText(labelRect, Qt::AlignCenter, label.text);
    }
}

QSize AccelerationVisualizationWidget::framebufferSizeInPixels(int logicalWidth, int logicalHeight) const
{
    const qreal devicePixelRatio = devicePixelRatioF();
    return QSize(qMax(1, qRound(logicalWidth * devicePixelRatio)),
                 qMax(1, qRound(logicalHeight * devicePixelRatio)));
}

void AccelerationVisualizationWidget::requestRender()
{
    if (isVisible()) {
        update();
    } else {
        m_pendingRender = true;
    }
}
