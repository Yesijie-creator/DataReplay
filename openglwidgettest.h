#ifndef OPENGLWIDGETTEST_H
#define OPENGLWIDGETTEST_H

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>

class OpenGLWidgetTest : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit OpenGLWidgetTest(QWidget *parent = nullptr);
    ~OpenGLWidgetTest() override;

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

private:
    QOpenGLShaderProgram m_program;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vertexBuffer;
    QPoint m_lastMousePos;
    float m_yawDeg = -25.0f;
    float m_pitchDeg = -18.0f;
};

#endif // OPENGLWIDGETTEST_H
