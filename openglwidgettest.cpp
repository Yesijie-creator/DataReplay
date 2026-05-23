#include "openglwidgettest.h"

#include <QDebug>
#include <QMatrix4x4>
#include <QMouseEvent>

namespace
{
struct VertexData
{
    QVector3D position;
    QVector3D color;
};

QVector<VertexData> buildCuboidVertices()
{
    const QVector3D c1(0.29f, 0.55f, 0.91f);
    const QVector3D c2(0.94f, 0.65f, 0.24f);
    const QVector3D c3(0.38f, 0.79f, 0.48f);
    const QVector3D c4(0.88f, 0.39f, 0.39f);
    const QVector3D c5(0.70f, 0.52f, 0.96f);
    const QVector3D c6(0.36f, 0.78f, 0.82f);

    const QVector3D lbf(-1.0f, -0.5f,  2.0f);
    const QVector3D rbf( 1.0f, -0.5f,  2.0f);
    const QVector3D rtf( 1.0f,  0.5f,  2.0f);
    const QVector3D ltf(-1.0f,  0.5f,  2.0f);
    const QVector3D lbb(-1.0f, -0.5f, -2.0f);
    const QVector3D rbb( 1.0f, -0.5f, -2.0f);
    const QVector3D rtb( 1.0f,  0.5f, -2.0f);
    const QVector3D ltb(-1.0f,  0.5f, -2.0f);

    return {
        { lbf, c1 }, { rbf, c1 }, { rtf, c1 }, { lbf, c1 }, { rtf, c1 }, { ltf, c1 },
        { rbb, c2 }, { lbb, c2 }, { ltb, c2 }, { rbb, c2 }, { ltb, c2 }, { rtb, c2 },
        { lbb, c3 }, { lbf, c3 }, { ltf, c3 }, { lbb, c3 }, { ltf, c3 }, { ltb, c3 },
        { rbf, c4 }, { rbb, c4 }, { rtb, c4 }, { rbf, c4 }, { rtb, c4 }, { rtf, c4 },
        { ltf, c5 }, { rtf, c5 }, { rtb, c5 }, { ltf, c5 }, { rtb, c5 }, { ltb, c5 },
        { lbb, c6 }, { rbb, c6 }, { rbf, c6 }, { lbb, c6 }, { rbf, c6 }, { lbf, c6 }
    };
}
}

OpenGLWidgetTest::OpenGLWidgetTest(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_vertexBuffer(QOpenGLBuffer::VertexBuffer)
{
    setMinimumSize(480, 360);
}

OpenGLWidgetTest::~OpenGLWidgetTest()
{
}

void OpenGLWidgetTest::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.12f, 0.16f, 0.22f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Vertex,
                                           "attribute vec3 position;\n"
                                           "attribute vec3 color;\n"
                                           "varying vec3 vColor;\n"
                                           "uniform mat4 mvp;\n"
                                           "void main()\n"
                                           "{\n"
                                           "    vColor = color;\n"
                                           "    gl_Position = mvp * vec4(position, 1.0);\n"
                                           "}\n")) {
        qDebug() << "OpenGL vertex shader error:" << m_program.log();
    }
    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Fragment,
                                           "varying vec3 vColor;\n"
                                           "void main()\n"
                                           "{\n"
                                           "    gl_FragColor = vec4(vColor, 1.0);\n"
                                           "}\n")) {
        qDebug() << "OpenGL fragment shader error:" << m_program.log();
    }
    if (!m_program.link()) {
        qDebug() << "OpenGL shader link error:" << m_program.log();
    }

    const QVector<VertexData> vertices = buildCuboidVertices();
    m_vao.create();
    m_vao.bind();

    m_vertexBuffer.create();
    m_vertexBuffer.bind();
    m_vertexBuffer.allocate(vertices.constData(), vertices.size() * static_cast<int>(sizeof(VertexData)));

    const int positionLocation = m_program.attributeLocation("position");
    const int colorLocation = m_program.attributeLocation("color");

    m_program.bind();
    m_program.enableAttributeArray(positionLocation);
    m_program.setAttributeBuffer(positionLocation, GL_FLOAT, offsetof(VertexData, position), 3, sizeof(VertexData));
    m_program.enableAttributeArray(colorLocation);
    m_program.setAttributeBuffer(colorLocation, GL_FLOAT, offsetof(VertexData, color), 3, sizeof(VertexData));
    m_program.release();

    m_vertexBuffer.release();
    m_vao.release();
}

void OpenGLWidgetTest::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 model;
    model.rotate(m_pitchDeg, 1.0f, 0.0f, 0.0f);
    model.rotate(m_yawDeg, 0.0f, 1.0f, 0.0f);

    QMatrix4x4 view;
    view.translate(0.0f, 0.0f, -7.0f);

    QMatrix4x4 projection;
    const float aspect = height() > 0 ? float(width()) / float(height()) : 1.0f;
    projection.perspective(45.0f, aspect, 0.1f, 100.0f);

    if (!m_program.isLinked()) {
        return;
    }

    m_program.bind();
    m_program.setUniformValue("mvp", projection * view * model);

    m_vao.bind();
    glDrawArrays(GL_TRIANGLES, 0, 36);
    m_vao.release();
    m_program.release();
}

void OpenGLWidgetTest::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void OpenGLWidgetTest::mousePressEvent(QMouseEvent *event)
{
    m_lastMousePos = event->pos();
}

void OpenGLWidgetTest::mouseMoveEvent(QMouseEvent *event)
{
    const QPoint delta = event->pos() - m_lastMousePos;
    m_lastMousePos = event->pos();

    if (event->buttons() & Qt::LeftButton) {
        m_yawDeg += delta.x() * 0.6f;
        m_pitchDeg += delta.y() * 0.6f;
        m_pitchDeg = qBound(-89.0f, m_pitchDeg, 89.0f);
        update();
    }
}
