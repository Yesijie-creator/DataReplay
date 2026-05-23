#include "mainwindow.h"

#include "accelerationvisualizationwidget.h"
#include "posesimulationwidget.h"
#include "qcustomplot.h"
#include "openglwidgettest.h"

#include <algorithm>
#include <QComboBox>
#include <QEvent>
#include <QFileDialog>
#include <QFontMetrics>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QAbstractItemView>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QScrollBar>
#include <QWheelEvent>
#include <QVBoxLayout>

namespace
{
QString formatNumber(double value, int precision = 3)
{
    return QString::number(value, 'f', precision);
}

static constexpr qint64 kDynamicFollowWindowSeconds = 10;
static constexpr qint64 kDynamicFollowRawSegmentMs = 500;
static constexpr qint64 kDynamicFollowWindowMs = kDynamicFollowWindowSeconds * 1000LL;
static constexpr qint64 kDynamicFollowSampledHistoryMs = kDynamicFollowWindowMs - kDynamicFollowRawSegmentMs;
static constexpr int kDynamicFollowHistoryBucketSize = 25;
static constexpr int kStaticOverviewMode = 0;
static constexpr int kDynamicFollowMode = 1;
static constexpr int kReplayTabIndex = 0;
static constexpr int kAccelerationTabIndex = 1;
static constexpr int kPoseTabIndex = 2;
static constexpr int kPlotCarouselIntervalMs = 3000;
static constexpr int kStaticOverviewPointsPerPixel = 2;
static constexpr int kStaticOverviewMaxBucketSamples = 4;
static constexpr int kStaticOverviewMinimumPointBudget = 512;
static constexpr double kSpeedAlertThresholdGs = 3.0;
static constexpr int kReplayPlotMinimumHeight = 240;

struct BucketPointRange
{
    bool hasPoint = false;
    int firstIndex = -1;
    int lastIndex = -1;
    int minIndex = -1;
    int maxIndex = -1;
    double minY = 0.0;
    double maxY = 0.0;
};

QPair<QVector<double>, QVector<double> > buildUniformlySampledRangeSeries(const QVector<double> &xData,
                                                                          const QVector<double> &yData,
                                                                          int startIndex,
                                                                          int endIndex,
                                                                          int maxPoints)
{
    QPair<QVector<double>, QVector<double> > result;
    if (xData.isEmpty() || yData.isEmpty() || xData.size() != yData.size() || startIndex > endIndex || maxPoints <= 0) {
        return result;
    }

    const int clampedStart = qBound(0, startIndex, xData.size() - 1);
    const int clampedEnd = qBound(0, endIndex, xData.size() - 1);
    if (clampedStart > clampedEnd) {
        return result;
    }

    const int count = clampedEnd - clampedStart + 1;
    if (count <= maxPoints) {
        result.first = xData.mid(clampedStart, count);
        result.second = yData.mid(clampedStart, count);
        return result;
    }

    result.first.reserve(maxPoints);
    result.second.reserve(maxPoints);
    const int denominator = qMax(1, maxPoints - 1);
    for (int sampleIndex = 0; sampleIndex < maxPoints; ++sampleIndex) {
        const int offset = (sampleIndex == maxPoints - 1)
                ? (count - 1)
                : static_cast<int>((static_cast<qint64>(sampleIndex) * (count - 1)) / denominator);
        const int sourceIndex = clampedStart + offset;
        result.first.push_back(xData.at(sourceIndex));
        result.second.push_back(yData.at(sourceIndex));
    }
    return result;
}

QVector<int> buildHistoryBucketExtremaIndices(const QVector<double> &yData,
                                              int startIndex,
                                              int endIndex,
                                              int bucketSize)
{
    QVector<int> result;
    if (yData.isEmpty() || startIndex > endIndex || bucketSize <= 0) {
        return result;
    }

    const int clampedStart = qBound(0, startIndex, yData.size() - 1);
    const int clampedEnd = qBound(0, endIndex, yData.size() - 1);
    if (clampedStart > clampedEnd) {
        return result;
    }

    const int count = clampedEnd - clampedStart + 1;
    result.reserve(((count + bucketSize - 1) / bucketSize) * 4);

    for (int bucketStart = clampedStart; bucketStart <= clampedEnd; bucketStart += bucketSize) {
        const int bucketEnd = qMin(clampedEnd, bucketStart + bucketSize - 1);
        int minIndex = bucketStart;
        int maxIndex = bucketStart;
        for (int index = bucketStart + 1; index <= bucketEnd; ++index) {
            if (yData.at(index) < yData.at(minIndex)) {
                minIndex = index;
            }
            if (yData.at(index) > yData.at(maxIndex)) {
                maxIndex = index;
            }
        }

        QVector<int> selectedIndices;
        selectedIndices.reserve(4);
        selectedIndices.push_back(bucketStart);
        selectedIndices.push_back(bucketEnd);
        selectedIndices.push_back(maxIndex);
        selectedIndices.push_back(minIndex);
        std::sort(selectedIndices.begin(), selectedIndices.end());
        selectedIndices.erase(std::unique(selectedIndices.begin(), selectedIndices.end()), selectedIndices.end());
        for (int selectedIndex : qAsConst(selectedIndices)) {
            result.push_back(selectedIndex);
        }
    }
    return result;
}

QVector<QVector<int> > buildHistoryBucketExtremaIndexCache(const QVector<double> &yData,
                                                           const QVector<int> &halfSecondStartIndices,
                                                           const QVector<int> &halfSecondEndIndices,
                                                           int bucketSize)
{
    QVector<QVector<int> > result;
    if (halfSecondStartIndices.size() != halfSecondEndIndices.size()) {
        return result;
    }

    result.fill(QVector<int>(), halfSecondStartIndices.size());
    for (int halfSecondBlock = 0; halfSecondBlock < halfSecondStartIndices.size(); ++halfSecondBlock) {
        const int blockStartIndex = halfSecondStartIndices.at(halfSecondBlock);
        const int blockEndIndex = halfSecondEndIndices.at(halfSecondBlock);
        if (blockStartIndex < 0 || blockEndIndex < blockStartIndex) {
            continue;
        }
        result[halfSecondBlock] = buildHistoryBucketExtremaIndices(yData,
                                                                   blockStartIndex,
                                                                   blockEndIndex,
                                                                   bucketSize);
    }
    return result;
}


double accelerationRangeFromSlider(int value)
{
    return static_cast<double>(value) / 100.0;
}

double poseExaggerationFromSlider(int value)
{
    return static_cast<double>(value) / 10.0;
}

double poseYawExaggerationFromSlider(int value)
{
    return static_cast<double>(value) / 10.0;
}

QColor plotSeriesColor(int index)
{
    switch (index) {
    case 0:
        return QColor(255, 107, 107);
    case 1:
        return QColor(46, 204, 113);
    case 2:
        return QColor(52, 152, 219);
    case 3:
        return QColor(155, 89, 182);
    default:
        return QColor(127, 140, 141);
    }
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    buildUi();

    m_playbackTimer = new QTimer(this);
    m_playbackTimer->setInterval(static_cast<int>(qMax<qint64>(1, m_sampleIntervalMs)));
    connect(m_playbackTimer, &QTimer::timeout, this, &MainWindow::playbackStep);

    m_infoCarouselTimer = new QTimer(this);
    m_infoCarouselTimer->setInterval(kPlotCarouselIntervalMs);
    connect(m_infoCarouselTimer, &QTimer::timeout, this, &MainWindow::advanceInfoCarousel);

    m_plotCarouselTimer = new QTimer(this);
    m_plotCarouselTimer->setInterval(kPlotCarouselIntervalMs);
    connect(m_plotCarouselTimer, &QTimer::timeout, this, &MainWindow::advancePlotCarousels);
}

MainWindow::~MainWindow()
{
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    if ((watched == m_topInfoLabel || watched == m_statusCarouselLabel) && event) {
        const bool hovered = event->type() == QEvent::Enter || event->type() == QEvent::HoverEnter;
        const bool left = event->type() == QEvent::Leave || event->type() == QEvent::HoverLeave;
        if (hovered || left) {
            const bool isHovered = hovered;
            if (watched == m_topInfoLabel) {
                m_topInfoHovered = isHovered;
            } else if (watched == m_statusCarouselLabel) {
                m_statusInfoHovered = isHovered;
            }
        }
    }

    QCustomPlot *plot = qobject_cast<QCustomPlot *>(watched);
    if (plot && event && event->type() == QEvent::Wheel && isReplayTabActive() && m_replayPlotScrollArea) {
        QScrollBar *verticalScrollBar = m_replayPlotScrollArea->verticalScrollBar();
        QWheelEvent *wheelEvent = static_cast<QWheelEvent *>(event);
        if (verticalScrollBar && wheelEvent) {
            int scrollDelta = 0;
            if (!wheelEvent->pixelDelta().isNull()) {
                scrollDelta = wheelEvent->pixelDelta().y();
            } else if (!wheelEvent->angleDelta().isNull()) {
                scrollDelta = (wheelEvent->angleDelta().y() * qMax(1, verticalScrollBar->singleStep())) / 120;
            }
            if (scrollDelta != 0) {
                verticalScrollBar->setValue(verticalScrollBar->value() - scrollDelta);
            }
            wheelEvent->accept();
            return true;
        }
    }

    if (plot && event && event->type() == QEvent::Resize && !m_samples.isEmpty()) {
        if (plot == m_accelPlot && isReplayTabActive() && isStaticOverviewMode(m_accelViewModeCombo)) {
            refreshSinglePlot(m_accelPlot,
                              m_accelDisplayCombo,
                              m_accelViewModeCombo,
                              QVector<QVector<double> >() << m_accelXData << m_accelYData << m_accelZData,
                              QVector<QVector<QVector<int> > >() << m_accelXHistorySampledIndices << m_accelYHistorySampledIndices << m_accelZHistorySampledIndices,
                              QVector<QString>() << QStringLiteral("Accel X") << QStringLiteral("Accel Y") << QStringLiteral("Accel Z"),
                              QStringLiteral("加速度 (g)"));
        } else if (plot == m_gyroPlot && isReplayTabActive() && isStaticOverviewMode(m_gyroViewModeCombo)) {
            refreshSinglePlot(m_gyroPlot,
                              m_gyroDisplayCombo,
                              m_gyroViewModeCombo,
                              QVector<QVector<double> >() << m_gyroXData << m_gyroYData << m_gyroZData,
                              QVector<QVector<QVector<int> > >() << m_gyroXHistorySampledIndices << m_gyroYHistorySampledIndices << m_gyroZHistorySampledIndices,
                              QVector<QString>() << QStringLiteral("Gyro X") << QStringLiteral("Gyro Y") << QStringLiteral("Gyro Z"),
                              QStringLiteral("角速度 (deg/s)"));
        } else if (plot == m_posePlot && isReplayTabActive() && isStaticOverviewMode(m_poseViewModeCombo)) {
            refreshSinglePlot(m_posePlot,
                              m_poseDisplayCombo,
                              m_poseViewModeCombo,
                              QVector<QVector<double> >() << m_rollData << m_pitchData << m_yawData,
                              QVector<QVector<QVector<int> > >() << m_rollHistorySampledIndices << m_pitchHistorySampledIndices << m_yawHistorySampledIndices,
                              QVector<QString>() << QStringLiteral("Roll") << QStringLiteral("Pitch") << QStringLiteral("Yaw"),
                              QStringLiteral("姿态角 (deg)"));
        } else if (plot == m_noisePlot && isReplayTabActive() && isStaticOverviewMode(m_noiseViewModeCombo)) {
            refreshNoisePlot();
        } else if (plot == m_speedPlot && isReplayTabActive() && isStaticOverviewMode(m_speedViewModeCombo)) {
            refreshSpeedPlot();
        }
    }
    return QMainWindow::eventFilter(watched, event);
}

void MainWindow::openDataFile()
{
    const QString filePath = QFileDialog::getOpenFileName(
                this,
                QStringLiteral("选择传感器文件"),
                QStringLiteral("c:/Users/15528/Documents/qt/DataReplay"),
                QStringLiteral("数据文件 (*.txt *.bin *.dat);;所有文件 (*.*)"));
    if (filePath.isEmpty()) {
        return;
    }

    const ParseResult result = m_parser.parseFile(filePath, m_hasMileageCorrectionTable ? &m_mileageCorrectionTable : nullptr);
    if (!result.errorMessage.isEmpty()) {
        m_currentSensorFilePath.clear();
        resetUiState();
        QMessageBox::warning(this, QStringLiteral("解析失败"), result.errorMessage);
        return;
    }

    m_currentSensorFilePath = filePath;
    setSamples(result, filePath);

    QString statusText = QStringLiteral("解析完成: C1=%1, C2=%2, 无效帧=%3")
            .arg(result.parsedC1)
            .arg(result.parsedC2)
            .arg(result.invalidFrames);
    m_topInfoItems = buildTopInfoItems(filePath, statusText, result.warningMessage);
    m_statusInfoItems = buildStatusInfoItems(statusText, result.warningMessage);
    m_topInfoIndex = 0;
    m_statusInfoIndex = 0;
    updateTopInfoDisplay();
    updateStatusInfoDisplay();
    if (statusBar()) {
        statusBar()->clearMessage();
    }
    updateInfoCarouselTimerState();

    if (result.sampleIntervalFallbackUsed) {
        const auto formatTimestamp = [](bool hasTimestamp, qint64 timestampMs) {
            if (!hasTimestamp) {
                return QStringLiteral("无有效时间戳");
            }
            return QDateTime::fromMSecsSinceEpoch(timestampMs, Qt::UTC)
                    .toString(QStringLiteral("yyyy-MM-dd HH:mm:ss UTC"));
        };
        QMessageBox::warning(
                    this,
                    QStringLiteral("步进计算失败"),
                    QStringLiteral("未能根据扫描到的首尾 C1 时间戳和 C2 数量计算有效步进，已回退为默认步进。\n"
                                   "首个 C1 时间戳：%1\n"
                                   "最后一个 C1 时间戳：%2\n"
                                   "C2 数量：%3")
                    .arg(formatTimestamp(result.hasFirstC1Timestamp, result.firstC1TimestampMs))
                    .arg(formatTimestamp(result.hasLastC1Timestamp, result.lastC1TimestampMs))
                    .arg(result.parsedC2));
    }
}

void MainWindow::openMileageCorrectionFile()
{
    const QString filePath = QFileDialog::getOpenFileName(
                this,
                QStringLiteral("选择里程校正文件"),
                QStringLiteral("c:/Users/15528/Documents/qt/DataReplay"),
                QStringLiteral("Excel 文件 (*.xlsx)"));
    if (filePath.isEmpty()) {
        return;
    }

    MileageCorrectionTable correctionTable;
    QString errorMessage;
    if (!m_parser.loadMileageCorrectionFile(filePath, &correctionTable, &errorMessage)) {
        QMessageBox::warning(this, QStringLiteral("读取失败"), errorMessage);
        return;
    }

    ParseResult result;
    if (!m_currentSensorFilePath.isEmpty()) {
        result = m_parser.parseFile(m_currentSensorFilePath, &correctionTable);
        if (!result.errorMessage.isEmpty()) {
            QMessageBox::warning(this, QStringLiteral("里程校正失败"), result.errorMessage);
            return;
        }
    }

    m_currentMileageCorrectionFilePath = filePath;
    m_mileageCorrectionTable = correctionTable;
    m_hasMileageCorrectionTable = true;

    if (!m_currentSensorFilePath.isEmpty()) {
        setSamples(result, m_currentSensorFilePath);
        const QString statusText = QStringLiteral("解析完成: C1=%1, C2=%2, 无效帧=%3")
                .arg(result.parsedC1)
                .arg(result.parsedC2)
                .arg(result.invalidFrames);
        m_topInfoItems = buildTopInfoItems(m_currentSensorFilePath, statusText, result.warningMessage);
        m_statusInfoItems = buildStatusInfoItems(statusText, result.warningMessage);
    } else {
        m_topInfoItems = QStringList()
                << QStringLiteral("未加载传感器文件")
                << QStringLiteral("里程校正文件: %1").arg(m_currentMileageCorrectionFilePath);
        m_statusInfoItems = QStringList()
                << QStringLiteral("里程校正文件已加载，等待加载传感器文件")
                << QStringLiteral("校正表工作表: %1，累计站点数: %2")
                   .arg(m_mileageCorrectionTable.sheetName)
                   .arg(m_mileageCorrectionTable.stations.size());
    }

    m_topInfoIndex = 0;
    m_statusInfoIndex = 0;
    updateTopInfoDisplay();
    updateStatusInfoDisplay();
    if (statusBar()) {
        statusBar()->clearMessage();
    }
    updateInfoCarouselTimerState();
}

void MainWindow::togglePlayback()
{
    if (m_playbackTimer && m_playbackTimer->isActive()) {
        pause();
    } else {
        play();
    }
}

void MainWindow::play()
{
    if (m_samples.isEmpty()) {
        return;
    }
    m_playbackTimer->start();
    updatePlaybackButtonText();
}

void MainWindow::pause()
{
    m_playbackTimer->stop();
    updatePlaybackButtonText();
}

void MainWindow::playbackStep()
{
    if (m_samples.isEmpty()) {
        pause();
        return;
    }

    const int step = qMax(1, qRound(m_playbackSpeed));
    int nextIndex = m_currentIndex + step;
    if (nextIndex >= m_samples.size()) {
        nextIndex = m_samples.size() - 1;
        pause();
    }

    updateUiForIndex(nextIndex);
}

void MainWindow::sliderChanged(int value)
{
    if (m_samples.isEmpty()) {
        return;
    }

    updateUiForIndex(value);
}

void MainWindow::speedChanged(int index)
{
    m_playbackSpeed = m_speedCombo->itemData(index).toDouble();
}

void MainWindow::plotDisplayModeChanged()
{
    if (isReplayTabActive()) {
        refreshPlots();
    }
}

void MainWindow::plotCarouselToggled(bool checked)
{
    Q_UNUSED(checked)
    updatePlotCarouselTimerState();
}

void MainWindow::plotModeSelectionChanged()
{
    if (m_plotCarouselProgrammaticChange) {
        return;
    }

    PlotCarouselContext *context = plotCarouselContextForObject(sender());
    if (!context || !context->carouselRadio || !context->carouselRadio->isChecked()) {
        return;
    }

    setPlotCarouselEnabled(context, false);
}

void MainWindow::tabChanged(int index)
{
    Q_UNUSED(index)
    refreshCurrentVisiblePage();
    if (isReplayTabActive()) {
        refreshPlots();
    }
    updatePlotCarouselTimerState();
}

void MainWindow::jumpToTime()
{
    if (m_samples.isEmpty()) {
        return;
    }

    const qint64 targetPlaybackMs = static_cast<qint64>(m_jumpHourSpin->value()) * 3600000LL
            + static_cast<qint64>(m_jumpMinuteSpin->value()) * 60000LL
            + static_cast<qint64>(m_jumpSecondSpin->value()) * 1000LL
            + static_cast<qint64>(m_jumpMillisecondSpin->value());

    const int targetIndex = findNearestSampleIndex(targetPlaybackMs);
    if (targetIndex >= 0) {
        updateUiForIndex(targetIndex);
    }
}

void MainWindow::jumpBackward15Seconds()
{
    jumpByOffsetMs(-15000);
}

void MainWindow::jumpForward30Seconds()
{
    jumpByOffsetMs(30000);
}

void MainWindow::accelerationRangeChanged(int value)
{
    const double rangeG = accelerationRangeFromSlider(value);
    if (m_accelerationRangeLabel) {
        m_accelerationRangeLabel->setText(QStringLiteral("球半径 = %1 g").arg(formatNumber(rangeG, 2)));
    }
    if (m_accelerationVisualizationView) {
        m_accelerationVisualizationView->setVisualRangeG(rangeG);
    }
}

void MainWindow::poseRollExaggerationChanged(int value)
{
    const double factor = poseExaggerationFromSlider(value);
    if (m_poseRollExaggerationLabel) {
        m_poseRollExaggerationLabel->setText(QStringLiteral("%1x").arg(formatNumber(factor, 1)));
    }
    if (m_poseSimulationView) {
        m_poseSimulationView->setRollExaggeration(factor);
    }
}

void MainWindow::posePitchExaggerationChanged(int value)
{
    const double factor = poseExaggerationFromSlider(value);
    if (m_posePitchExaggerationLabel) {
        m_posePitchExaggerationLabel->setText(QStringLiteral("%1x").arg(formatNumber(factor, 1)));
    }
    if (m_poseSimulationView) {
        m_poseSimulationView->setPitchExaggeration(factor);
    }
}

void MainWindow::poseYawExaggerationChanged(int value)
{
    const double factor = poseYawExaggerationFromSlider(value);
    if (m_poseYawExaggerationLabel) {
        m_poseYawExaggerationLabel->setText(QStringLiteral("%1x").arg(formatNumber(factor, 1)));
    }
    if (m_poseSimulationView) {
        m_poseSimulationView->setYawExaggeration(factor);
    }
}

void MainWindow::buildUi()
{
    resize(1480, 920);
    setWindowTitle(QStringLiteral("原始数据回放"));

    QWidget *central = new QWidget(this);
    setCentralWidget(central);

    QVBoxLayout *rootLayout = new QVBoxLayout(central);
    rootLayout->setContentsMargins(10, 10, 10, 10);
    rootLayout->setSpacing(10);

    QHBoxLayout *toolbarLayout = new QHBoxLayout;
    m_openButton = new QPushButton(QStringLiteral("加载传感器文件"), this);
    m_openMileageCorrectionButton = new QPushButton(QStringLiteral("加载里程校正文件"), this);
    m_playButton = new QPushButton(QStringLiteral("播放"), this);
    m_speedCombo = new QComboBox(this);
    m_speedCombo->addItem(QStringLiteral("0.5x"), 0.5);
    m_speedCombo->addItem(QStringLiteral("1.0x"), 1.0);
    m_speedCombo->addItem(QStringLiteral("2.0x"), 2.0);
    m_speedCombo->addItem(QStringLiteral("4.0x"), 4.0);
    m_speedCombo->addItem(QStringLiteral("5.0x"), 5.0);
    m_speedCombo->addItem(QStringLiteral("50.0x"), 50.0);
    m_speedCombo->addItem(QStringLiteral("100.0x"), 100.0);
    m_speedCombo->addItem(QStringLiteral("500.0x"), 500.0);
    m_speedCombo->setCurrentIndex(4);
    m_playbackSpeed = m_speedCombo->currentData().toDouble();
    connect(m_openButton, &QPushButton::clicked, this, &MainWindow::openDataFile);
    connect(m_openMileageCorrectionButton, &QPushButton::clicked, this, &MainWindow::openMileageCorrectionFile);
    connect(m_playButton, &QPushButton::clicked, this, &MainWindow::togglePlayback);
    connect(m_speedCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::speedChanged);

    m_topInfoLabel = new QLabel(QStringLiteral("未加载传感器文件"), this);
    m_topInfoLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    m_topInfoLabel->setWordWrap(false);
    m_topInfoLabel->installEventFilter(this);
    toolbarLayout->addWidget(m_openButton);
    toolbarLayout->addWidget(m_openMileageCorrectionButton);
    toolbarLayout->addWidget(m_playButton);
    toolbarLayout->addWidget(new QLabel(QStringLiteral("倍速"), this));
    toolbarLayout->addWidget(m_speedCombo);
    toolbarLayout->addSpacing(12);
    toolbarLayout->addWidget(m_topInfoLabel, 1);
    rootLayout->addLayout(toolbarLayout);

    QVBoxLayout *timelineSectionLayout = new QVBoxLayout;
    timelineSectionLayout->setSpacing(6);

    QHBoxLayout *timelineLayout = new QHBoxLayout;
    m_currentTimeLabel = new QLabel(QStringLiteral("当前时刻: --"), this);
    m_poseLabel = new QLabel(QStringLiteral("姿态: --"), this);
    m_timelineSlider = new QSlider(Qt::Horizontal, this);
    m_timelineSlider->setRange(0, 0);
    expandLabelMinimumWidth(m_currentTimeLabel, &m_currentTimeMinWidth, m_currentTimeLabel->text());
    expandLabelMinimumWidth(m_poseLabel, &m_poseMinWidth, QStringLiteral("姿态: 当前时刻不可用"));
    connect(m_timelineSlider, &QSlider::valueChanged, this, &MainWindow::sliderChanged);
    timelineLayout->addWidget(m_currentTimeLabel);
    timelineLayout->addWidget(m_timelineSlider, 1);
    timelineLayout->addWidget(m_poseLabel);
    timelineSectionLayout->addLayout(timelineLayout);

    QHBoxLayout *jumpLayout = new QHBoxLayout;
    jumpLayout->addStretch();
    m_frameCostLabel = new QLabel(this);
    expandLabelMinimumWidth(m_frameCostLabel, &m_frameCostMinWidth, QStringLiteral("单帧耗时: 00000.0 ms"));
    jumpLayout->addWidget(m_frameCostLabel);
    jumpLayout->addSpacing(50);
    jumpLayout->addWidget(new QLabel(QStringLiteral("前往指定时刻"), this));

    m_jumpHourSpin = new QSpinBox(this);
    m_jumpMinuteSpin = new QSpinBox(this);
    m_jumpSecondSpin = new QSpinBox(this);
    m_jumpMillisecondSpin = new QSpinBox(this);
    m_jumpButton = new QPushButton(QStringLiteral("前往"), this);
    m_jumpBackwardButton = new QPushButton(QStringLiteral("后退15S"), this);
    m_jumpForwardButton = new QPushButton(QStringLiteral("快进30S"), this);

    const QList<QSpinBox *> jumpSpinBoxes = {
        m_jumpHourSpin,
        m_jumpMinuteSpin,
        m_jumpSecondSpin,
        m_jumpMillisecondSpin
    };
    for (QSpinBox *spinBox : jumpSpinBoxes) {
        spinBox->setAccelerated(true);
        spinBox->setButtonSymbols(QAbstractSpinBox::PlusMinus);
    }

    m_jumpHourSpin->setRange(0, 0);
    m_jumpMinuteSpin->setRange(0, 59);
    m_jumpSecondSpin->setRange(0, 59);
    m_jumpMillisecondSpin->setRange(0, 999);
    m_jumpMillisecondSpin->setSingleStep(static_cast<int>(qMax<qint64>(1, m_sampleIntervalMs)));

    jumpLayout->addWidget(m_jumpHourSpin);
    jumpLayout->addWidget(new QLabel(QStringLiteral("时"), this));
    jumpLayout->addWidget(m_jumpMinuteSpin);
    jumpLayout->addWidget(new QLabel(QStringLiteral("分"), this));
    jumpLayout->addWidget(m_jumpSecondSpin);
    jumpLayout->addWidget(new QLabel(QStringLiteral("秒"), this));
    jumpLayout->addWidget(m_jumpMillisecondSpin);
    jumpLayout->addWidget(new QLabel(QStringLiteral("毫秒"), this));
    jumpLayout->addWidget(m_jumpButton);
    jumpLayout->addSpacing(50);
    jumpLayout->addWidget(m_jumpBackwardButton);
    jumpLayout->addWidget(m_jumpForwardButton);
    jumpLayout->addStretch();
    connect(m_jumpBackwardButton, &QPushButton::clicked, this, &MainWindow::jumpBackward15Seconds);
    connect(m_jumpForwardButton, &QPushButton::clicked, this, &MainWindow::jumpForward30Seconds);
    connect(m_jumpButton, &QPushButton::clicked, this, &MainWindow::jumpToTime);

    timelineSectionLayout->addLayout(jumpLayout);
    rootLayout->addLayout(timelineSectionLayout);

    m_mainTabs = new QTabWidget(this);
    m_replayPage = new QWidget(m_mainTabs);
    QHBoxLayout *parameterLayout = new QHBoxLayout(m_replayPage);
    parameterLayout->setContentsMargins(0, 0, 0, 0);
    parameterLayout->setSpacing(10);
    QVBoxLayout *leftPanelLayout = new QVBoxLayout;
    leftPanelLayout->setSpacing(10);

    QGroupBox *valueBox = new QGroupBox(QStringLiteral("当前参数"), m_replayPage);
    QVBoxLayout *valueLayout = new QVBoxLayout(valueBox);
    m_valueTable = new QTableWidget(10, 2, valueBox);
    m_valueTable->setHorizontalHeaderLabels(QStringList() << QStringLiteral("字段") << QStringLiteral("值"));
    m_valueTable->horizontalHeader()->setStretchLastSection(true);
    m_valueTable->verticalHeader()->setVisible(false);
    m_valueTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_valueTable->setSelectionMode(QAbstractItemView::NoSelection);
    m_valueTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    for (int row = 0; row < m_valueTable->rowCount(); ++row) {
        m_valueTable->setItem(row, 0, new QTableWidgetItem);
        m_valueTable->setItem(row, 1, new QTableWidgetItem);
    }
    valueBox->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    valueLayout->addWidget(m_valueTable);

    QGroupBox *solutionBox = new QGroupBox(QStringLiteral("解算参数"), m_replayPage);
    QVBoxLayout *solutionLayout = new QVBoxLayout(solutionBox);
    m_solutionTable = new QTableWidget(2, 2, solutionBox);
    m_solutionTable->setHorizontalHeaderLabels(QStringList() << QStringLiteral("字段") << QStringLiteral("值"));
    m_solutionTable->horizontalHeader()->setStretchLastSection(true);
    m_solutionTable->verticalHeader()->setVisible(false);
    m_solutionTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_solutionTable->setSelectionMode(QAbstractItemView::NoSelection);
    m_solutionTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    for (int row = 0; row < m_solutionTable->rowCount(); ++row) {
        m_solutionTable->setItem(row, 0, new QTableWidgetItem);
        m_solutionTable->setItem(row, 1, new QTableWidgetItem);
    }
    solutionBox->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    solutionLayout->addWidget(m_solutionTable);
    leftPanelLayout->addWidget(valueBox, 4);
    leftPanelLayout->addWidget(solutionBox, 1);

    m_replayPlotContainer = new QWidget(m_replayPage);
    QVBoxLayout *plotLayout = new QVBoxLayout(m_replayPlotContainer);
    plotLayout->setSpacing(10);
    plotLayout->setContentsMargins(0, 0, 0, 0);

    QGroupBox *accelBox = new QGroupBox(QStringLiteral("三轴加速度 (g)"), m_replayPage);
    QVBoxLayout *accelLayout = new QVBoxLayout(accelBox);
    QHBoxLayout *accelToolbarLayout = new QHBoxLayout;
    accelToolbarLayout->addWidget(new QLabel(QStringLiteral("视图模式"), accelBox));
    m_accelViewModeCombo = new QComboBox(accelBox);
    m_accelViewModeCombo->addItem(QStringLiteral("全局静态总览"), kStaticOverviewMode);
    m_accelViewModeCombo->addItem(QStringLiteral("局部动态跟随"), kDynamicFollowMode);
    m_accelViewModeCombo->setCurrentIndex(1);
    accelToolbarLayout->addWidget(m_accelViewModeCombo);
    accelToolbarLayout->addSpacing(12);
    accelToolbarLayout->addWidget(new QLabel(QStringLiteral("显示模式"), accelBox));
    m_accelDisplayCombo = new QComboBox(accelBox);
    m_accelDisplayCombo->addItems(QStringList()
                                  << QStringLiteral("全部显示")
                                  << QStringLiteral("X轴")
                                  << QStringLiteral("Y轴")
                                  << QStringLiteral("Z轴"));
    accelToolbarLayout->addWidget(m_accelDisplayCombo);
    accelToolbarLayout->addSpacing(12);
    m_accelCarouselRadio = new QRadioButton(QStringLiteral("轮播"), accelBox);
    m_accelCarouselRadio->setChecked(true);
    accelToolbarLayout->addWidget(m_accelCarouselRadio);
    accelToolbarLayout->addStretch();
    m_accelPlot = new QCustomPlot(accelBox);
    configurePlot(m_accelPlot);
    m_accelPlot->installEventFilter(this);
    accelLayout->addLayout(accelToolbarLayout);
    accelLayout->addWidget(m_accelPlot);
    accelBox->setMinimumHeight(kReplayPlotMinimumHeight);

    QGroupBox *gyroBox = new QGroupBox(QStringLiteral("三轴角速度 (deg/s)"), m_replayPage);
    QVBoxLayout *gyroLayout = new QVBoxLayout(gyroBox);
    QHBoxLayout *gyroToolbarLayout = new QHBoxLayout;
    gyroToolbarLayout->addWidget(new QLabel(QStringLiteral("视图模式"), gyroBox));
    m_gyroViewModeCombo = new QComboBox(gyroBox);
    m_gyroViewModeCombo->addItem(QStringLiteral("全局静态总览"), kStaticOverviewMode);
    m_gyroViewModeCombo->addItem(QStringLiteral("局部动态跟随"), kDynamicFollowMode);
    m_gyroViewModeCombo->setCurrentIndex(1);
    gyroToolbarLayout->addWidget(m_gyroViewModeCombo);
    gyroToolbarLayout->addSpacing(12);
    gyroToolbarLayout->addWidget(new QLabel(QStringLiteral("显示模式"), gyroBox));
    m_gyroDisplayCombo = new QComboBox(gyroBox);
    m_gyroDisplayCombo->addItems(QStringList()
                                 << QStringLiteral("全部显示")
                                 << QStringLiteral("X轴 (Yaw)")
                                 << QStringLiteral("Y轴 (Pitch)")
                                 << QStringLiteral("Z轴 (Roll)"));
    gyroToolbarLayout->addWidget(m_gyroDisplayCombo);
    gyroToolbarLayout->addSpacing(12);
    m_gyroCarouselRadio = new QRadioButton(QStringLiteral("轮播"), gyroBox);
    m_gyroCarouselRadio->setChecked(true);
    gyroToolbarLayout->addWidget(m_gyroCarouselRadio);
    gyroToolbarLayout->addStretch();
    m_gyroPlot = new QCustomPlot(gyroBox);
    configurePlot(m_gyroPlot);
    m_gyroPlot->installEventFilter(this);
    gyroLayout->addLayout(gyroToolbarLayout);
    gyroLayout->addWidget(m_gyroPlot);
    gyroBox->setMinimumHeight(kReplayPlotMinimumHeight);

    QGroupBox *poseCurveBox = new QGroupBox(QStringLiteral("姿态角曲线 (deg)"), m_replayPage);
    QVBoxLayout *poseCurveLayout = new QVBoxLayout(poseCurveBox);
    QHBoxLayout *poseToolbarLayout = new QHBoxLayout;
    poseToolbarLayout->addWidget(new QLabel(QStringLiteral("视图模式"), poseCurveBox));
    m_poseViewModeCombo = new QComboBox(poseCurveBox);
    m_poseViewModeCombo->addItem(QStringLiteral("全局静态总览"), kStaticOverviewMode);
    m_poseViewModeCombo->addItem(QStringLiteral("局部动态跟随"), kDynamicFollowMode);
    m_poseViewModeCombo->setCurrentIndex(1);
    poseToolbarLayout->addWidget(m_poseViewModeCombo);
    poseToolbarLayout->addSpacing(12);
    poseToolbarLayout->addWidget(new QLabel(QStringLiteral("显示模式"), poseCurveBox));
    m_poseDisplayCombo = new QComboBox(poseCurveBox);
    m_poseDisplayCombo->addItems(QStringList()
                                 << QStringLiteral("全部显示")
                                 << QStringLiteral("Roll")
                                 << QStringLiteral("Pitch")
                                 << QStringLiteral("Yaw"));
    poseToolbarLayout->addWidget(m_poseDisplayCombo);
    poseToolbarLayout->addSpacing(12);
    m_poseCarouselRadio = new QRadioButton(QStringLiteral("轮播"), poseCurveBox);
    m_poseCarouselRadio->setChecked(true);
    poseToolbarLayout->addWidget(m_poseCarouselRadio);
    poseToolbarLayout->addStretch();
    m_posePlot = new QCustomPlot(poseCurveBox);
    configurePlot(m_posePlot);
    m_posePlot->installEventFilter(this);
    poseCurveLayout->addLayout(poseToolbarLayout);
    poseCurveLayout->addWidget(m_posePlot);
    poseCurveBox->setMinimumHeight(kReplayPlotMinimumHeight);

    m_noisePlotBox = new QGroupBox(QStringLiteral("噪声 (dBA)"), m_replayPage);
    QVBoxLayout *noiseLayout = new QVBoxLayout(m_noisePlotBox);
    QHBoxLayout *noiseToolbarLayout = new QHBoxLayout;
    noiseToolbarLayout->addWidget(new QLabel(QStringLiteral("视图模式"), m_noisePlotBox));
    m_noiseViewModeCombo = new QComboBox(m_noisePlotBox);
    m_noiseViewModeCombo->addItem(QStringLiteral("全局静态总览"), kStaticOverviewMode);
    m_noiseViewModeCombo->addItem(QStringLiteral("局部动态跟随"), kDynamicFollowMode);
    m_noiseViewModeCombo->setCurrentIndex(1);
    noiseToolbarLayout->addWidget(m_noiseViewModeCombo);
    noiseToolbarLayout->addSpacing(12);
    m_noiseCarouselRadio = new QRadioButton(QStringLiteral("轮播"), m_noisePlotBox);
    m_noiseCarouselRadio->setChecked(true);
    noiseToolbarLayout->addWidget(m_noiseCarouselRadio);
    noiseToolbarLayout->addStretch();
    m_noisePlot = new QCustomPlot(m_noisePlotBox);
    configurePlot(m_noisePlot);
    m_noisePlot->installEventFilter(this);
    noiseLayout->addLayout(noiseToolbarLayout);
    noiseLayout->addWidget(m_noisePlot);
    m_noisePlotBox->setMinimumHeight(kReplayPlotMinimumHeight);

    m_speedPlotBox = new QGroupBox(QStringLiteral("速度 (解算 Z, g*s)"), m_replayPage);
    QVBoxLayout *speedLayout = new QVBoxLayout(m_speedPlotBox);
    QHBoxLayout *speedToolbarLayout = new QHBoxLayout;
    speedToolbarLayout->addWidget(new QLabel(QStringLiteral("视图模式"), m_speedPlotBox));
    m_speedViewModeCombo = new QComboBox(m_speedPlotBox);
    m_speedViewModeCombo->addItem(QStringLiteral("全局静态总览"), kStaticOverviewMode);
    m_speedViewModeCombo->addItem(QStringLiteral("局部动态跟随"), kDynamicFollowMode);
    m_speedViewModeCombo->setCurrentIndex(1);
    speedToolbarLayout->addWidget(m_speedViewModeCombo);
    speedToolbarLayout->addSpacing(12);
    m_speedCarouselRadio = new QRadioButton(QStringLiteral("轮播"), m_speedPlotBox);
    m_speedCarouselRadio->setChecked(true);
    speedToolbarLayout->addWidget(m_speedCarouselRadio);
    speedToolbarLayout->addStretch();
    m_speedPlot = new QCustomPlot(m_speedPlotBox);
    configurePlot(m_speedPlot);
    m_speedPlot->installEventFilter(this);
    speedLayout->addLayout(speedToolbarLayout);
    speedLayout->addWidget(m_speedPlot);
    m_speedPlotBox->setMinimumHeight(kReplayPlotMinimumHeight);

    connect(m_accelViewModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotDisplayModeChanged);
    connect(m_accelDisplayCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotDisplayModeChanged);
    connect(m_gyroViewModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotDisplayModeChanged);
    connect(m_gyroDisplayCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotDisplayModeChanged);
    connect(m_poseViewModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotDisplayModeChanged);
    connect(m_poseDisplayCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotDisplayModeChanged);
    connect(m_noiseViewModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotDisplayModeChanged);
    connect(m_speedViewModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotDisplayModeChanged);
    connect(m_accelViewModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotModeSelectionChanged);
    connect(m_accelDisplayCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotModeSelectionChanged);
    connect(m_gyroViewModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotModeSelectionChanged);
    connect(m_gyroDisplayCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotModeSelectionChanged);
    connect(m_poseViewModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotModeSelectionChanged);
    connect(m_poseDisplayCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotModeSelectionChanged);
    connect(m_noiseViewModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotModeSelectionChanged);
    connect(m_speedViewModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::plotModeSelectionChanged);
    connect(m_accelCarouselRadio, &QRadioButton::toggled, this, &MainWindow::plotCarouselToggled);
    connect(m_gyroCarouselRadio, &QRadioButton::toggled, this, &MainWindow::plotCarouselToggled);
    connect(m_poseCarouselRadio, &QRadioButton::toggled, this, &MainWindow::plotCarouselToggled);
    connect(m_noiseCarouselRadio, &QRadioButton::toggled, this, &MainWindow::plotCarouselToggled);
    connect(m_speedCarouselRadio, &QRadioButton::toggled, this, &MainWindow::plotCarouselToggled);

    plotLayout->addWidget(accelBox);
    plotLayout->addWidget(gyroBox);
    plotLayout->addWidget(poseCurveBox);
    plotLayout->addWidget(m_noisePlotBox);
    plotLayout->addWidget(m_speedPlotBox);
    plotLayout->addStretch();

    m_replayPlotScrollArea = new QScrollArea(m_replayPage);
    m_replayPlotScrollArea->setWidgetResizable(true);
    m_replayPlotScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_replayPlotScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_replayPlotScrollArea->setFrameShape(QFrame::NoFrame);
    m_replayPlotScrollArea->setWidget(m_replayPlotContainer);
    connect(m_replayPlotScrollArea->verticalScrollBar(), &QScrollBar::valueChanged, this, [this]() {
        m_suspendReplayPlotRendering = true;
        ++m_replayPlotScrollSuspendToken;
        const qint64 suspendToken = m_replayPlotScrollSuspendToken;
        QTimer::singleShot(120, this, [this, suspendToken]() {
            if (suspendToken != m_replayPlotScrollSuspendToken) {
                return;
            }
            m_suspendReplayPlotRendering = false;
            if (isReplayTabActive() && !m_samples.isEmpty()) {
                refreshPlots();
            }
        });
    });

    parameterLayout->addLayout(leftPanelLayout, 2);
    parameterLayout->addWidget(m_replayPlotScrollArea, 5);

    QWidget *accelerationPage = new QWidget(m_mainTabs);
    QVBoxLayout *accelerationLayout = new QVBoxLayout(accelerationPage);
    accelerationLayout->setContentsMargins(0, 0, 0, 0);
    accelerationLayout->setSpacing(8);
    m_accelerationVisualizationView = new AccelerationVisualizationWidget(accelerationPage);
    accelerationLayout->addWidget(m_accelerationVisualizationView);
    QHBoxLayout *accelerationControlLayout = new QHBoxLayout;
    QWidget *accelerationControlBar = new QWidget(accelerationPage);
    accelerationControlBar->setFixedHeight(35);
    accelerationControlBar->setLayout(accelerationControlLayout);
    QLabel *accelerationRangeTitleLabel = new QLabel(QStringLiteral("球体量程"), accelerationPage);
    accelerationRangeTitleLabel->setMaximumHeight(35);
    accelerationControlLayout->addWidget(accelerationRangeTitleLabel);
    m_accelerationRangeSlider = new QSlider(Qt::Horizontal, accelerationPage);
    m_accelerationRangeSlider->setRange(10, 100);
    m_accelerationRangeSlider->setValue(10);
    m_accelerationRangeSlider->setMaximumHeight(35);
    m_accelerationRangeLabel = new QLabel(accelerationPage);
    m_accelerationRangeLabel->setMaximumHeight(35);
    accelerationControlLayout->addWidget(m_accelerationRangeSlider, 1);
    accelerationControlLayout->addWidget(m_accelerationRangeLabel);
    accelerationLayout->addWidget(accelerationControlBar);
    connect(m_accelerationRangeSlider, &QSlider::valueChanged, this, &MainWindow::accelerationRangeChanged);

    QWidget *poseSimulationPage = new QWidget(m_mainTabs);
    QVBoxLayout *poseSimulationLayout = new QVBoxLayout(poseSimulationPage);
    poseSimulationLayout->setContentsMargins(0, 0, 0, 0);
    poseSimulationLayout->setSpacing(8);
    m_poseSimulationView = new PoseSimulationWidget(poseSimulationPage);
    poseSimulationLayout->addWidget(m_poseSimulationView);
    QGridLayout *poseControlLayout = new QGridLayout;
    QWidget *poseControlBar = new QWidget(poseSimulationPage);
    poseControlBar->setLayout(poseControlLayout);
    poseControlLayout->setContentsMargins(6, 4, 6, 4);
    poseControlLayout->setHorizontalSpacing(8);
    poseControlLayout->setVerticalSpacing(4);
    poseControlBar->setMaximumHeight(113);

    QLabel *rollTitleLabel = new QLabel(QStringLiteral("Roll 左右倾斜/抖动"), poseSimulationPage);
    rollTitleLabel->setMaximumHeight(35);
    poseControlLayout->addWidget(rollTitleLabel, 0, 0);
    m_poseRollExaggerationSlider = new QSlider(Qt::Horizontal, poseSimulationPage);
    m_poseRollExaggerationSlider->setRange(10, 500);
    m_poseRollExaggerationSlider->setValue(10);
    m_poseRollExaggerationSlider->setMaximumHeight(35);
    m_poseRollExaggerationLabel = new QLabel(poseSimulationPage);
    m_poseRollExaggerationLabel->setMaximumHeight(35);
    poseControlLayout->addWidget(m_poseRollExaggerationSlider, 0, 1);
    poseControlLayout->addWidget(m_poseRollExaggerationLabel, 0, 2);

    QLabel *pitchTitleLabel = new QLabel(QStringLiteral("Pitch 俯仰"), poseSimulationPage);
    pitchTitleLabel->setMaximumHeight(35);
    poseControlLayout->addWidget(pitchTitleLabel, 1, 0);
    m_posePitchExaggerationSlider = new QSlider(Qt::Horizontal, poseSimulationPage);
    m_posePitchExaggerationSlider->setRange(10, 500);
    m_posePitchExaggerationSlider->setValue(10);
    m_posePitchExaggerationSlider->setMaximumHeight(35);
    m_posePitchExaggerationLabel = new QLabel(poseSimulationPage);
    m_posePitchExaggerationLabel->setMaximumHeight(35);
    poseControlLayout->addWidget(m_posePitchExaggerationSlider, 1, 1);
    poseControlLayout->addWidget(m_posePitchExaggerationLabel, 1, 2);

    QLabel *yawTitleLabel = new QLabel(QStringLiteral("Yaw 偏航"), poseSimulationPage);
    yawTitleLabel->setMaximumHeight(35);
    poseControlLayout->addWidget(yawTitleLabel, 2, 0);
    m_poseYawExaggerationSlider = new QSlider(Qt::Horizontal, poseSimulationPage);
    m_poseYawExaggerationSlider->setRange(10, 30);
    m_poseYawExaggerationSlider->setValue(10);
    m_poseYawExaggerationSlider->setMaximumHeight(35);
    m_poseYawExaggerationLabel = new QLabel(poseSimulationPage);
    m_poseYawExaggerationLabel->setMaximumHeight(35);
    poseControlLayout->addWidget(m_poseYawExaggerationSlider, 2, 1);
    poseControlLayout->addWidget(m_poseYawExaggerationLabel, 2, 2);
    poseSimulationLayout->addWidget(poseControlBar);
    connect(m_poseRollExaggerationSlider, &QSlider::valueChanged, this, &MainWindow::poseRollExaggerationChanged);
    connect(m_posePitchExaggerationSlider, &QSlider::valueChanged, this, &MainWindow::posePitchExaggerationChanged);
    connect(m_poseYawExaggerationSlider, &QSlider::valueChanged, this, &MainWindow::poseYawExaggerationChanged);

    QWidget *openGlWidgetTestPage = new QWidget(m_mainTabs);
    QVBoxLayout *openGlWidgetTestLayout = new QVBoxLayout(openGlWidgetTestPage);
    openGlWidgetTestLayout->setContentsMargins(0, 0, 0, 0);
    openGlWidgetTestLayout->setSpacing(0);
    m_openGlWidgetTestView = new OpenGLWidgetTest(openGlWidgetTestPage);
    openGlWidgetTestLayout->addWidget(m_openGlWidgetTestView);

    m_mainTabs->addTab(m_replayPage, QStringLiteral("数据回放"));
    m_mainTabs->addTab(accelerationPage, QStringLiteral("加速度可视化"));
    m_mainTabs->addTab(poseSimulationPage, QStringLiteral("姿态模拟"));
    m_mainTabs->addTab(openGlWidgetTestPage, QStringLiteral("OpenGL长方体测试"));
    connect(m_mainTabs, &QTabWidget::currentChanged, this, &MainWindow::tabChanged);
    rootLayout->addWidget(m_mainTabs, 1);

    accelerationRangeChanged(m_accelerationRangeSlider->value());
    poseRollExaggerationChanged(m_poseRollExaggerationSlider->value());
    posePitchExaggerationChanged(m_posePitchExaggerationSlider->value());
    poseYawExaggerationChanged(m_poseYawExaggerationSlider->value());
    m_accelCarouselContext.viewModeCombo = m_accelViewModeCombo;
    m_accelCarouselContext.displayCombo = m_accelDisplayCombo;
    m_accelCarouselContext.carouselRadio = m_accelCarouselRadio;
    m_gyroCarouselContext.viewModeCombo = m_gyroViewModeCombo;
    m_gyroCarouselContext.displayCombo = m_gyroDisplayCombo;
    m_gyroCarouselContext.carouselRadio = m_gyroCarouselRadio;
    m_poseCarouselContext.viewModeCombo = m_poseViewModeCombo;
    m_poseCarouselContext.displayCombo = m_poseDisplayCombo;
    m_poseCarouselContext.carouselRadio = m_poseCarouselRadio;
    m_noiseCarouselContext.viewModeCombo = m_noiseViewModeCombo;
    m_noiseCarouselContext.carouselRadio = m_noiseCarouselRadio;
    m_speedCarouselContext.viewModeCombo = m_speedViewModeCombo;
    m_speedCarouselContext.carouselRadio = m_speedCarouselRadio;
    initializePlotCarousels();
    updatePlaybackButtonText();
    updateFrameCostLabel();
    m_statusCarouselLabel = new QLabel(QStringLiteral("等待加载传感器文件"), this);
    m_statusCarouselLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    m_statusCarouselLabel->setWordWrap(false);
    m_statusCarouselLabel->installEventFilter(this);
    statusBar()->addPermanentWidget(m_statusCarouselLabel, 1);
    resetInfoCarouselState();
    setControlsEnabled(false);
}

void MainWindow::setSamples(const ParseResult &result, const QString &filePath)
{
    m_samples = result.samples;
    m_poseSceneGeometry = result.poseSceneGeometry;
    m_sampleIntervalMs = qMax<qint64>(1, result.resolvedSampleIntervalMs);
    m_sampleIntervalSeconds = result.resolvedSampleIntervalSeconds > 0.0
            ? result.resolvedSampleIntervalSeconds
            : static_cast<double>(m_sampleIntervalMs) / 1000.0;
    m_frameCostWarningThresholdMs = static_cast<double>(m_sampleIntervalMs);
    if (m_playbackTimer) {
        m_playbackTimer->setInterval(static_cast<int>(m_sampleIntervalMs));
    }
    if (m_jumpMillisecondSpin) {
        m_jumpMillisecondSpin->setSingleStep(static_cast<int>(m_sampleIntervalMs));
    }

    if (!m_samples.isEmpty()) {
        expandLabelMinimumWidth(m_currentTimeLabel,
                                &m_currentTimeMinWidth,
                                QStringLiteral("当前时刻: %1").arg(formatTimestamp(m_samples.constFirst())));
        expandLabelMinimumWidth(m_currentTimeLabel,
                                &m_currentTimeMinWidth,
                                QStringLiteral("当前时刻: %1").arg(formatTimestamp(m_samples.constLast())));

        double minRollDeg = 0.0;
        double maxRollDeg = 0.0;
        double minPitchDeg = 0.0;
        double maxPitchDeg = 0.0;
        double minYawDeg = 0.0;
        double maxYawDeg = 0.0;
        bool hasPoseSample = false;
        for (const ReplaySample &sample : qAsConst(m_samples)) {
            if (!sample.poseValid) {
                continue;
            }

            if (!hasPoseSample) {
                minRollDeg = maxRollDeg = sample.rollDeg;
                minPitchDeg = maxPitchDeg = sample.pitchDeg;
                minYawDeg = maxYawDeg = sample.yawDeg;
                hasPoseSample = true;
                continue;
            }

            minRollDeg = qMin(minRollDeg, sample.rollDeg);
            maxRollDeg = qMax(maxRollDeg, sample.rollDeg);
            minPitchDeg = qMin(minPitchDeg, sample.pitchDeg);
            maxPitchDeg = qMax(maxPitchDeg, sample.pitchDeg);
            minYawDeg = qMin(minYawDeg, sample.yawDeg);
            maxYawDeg = qMax(maxYawDeg, sample.yawDeg);
        }

        expandLabelMinimumWidth(m_poseLabel, &m_poseMinWidth, QStringLiteral("姿态: 当前时刻不可用"));
        if (hasPoseSample) {
            const QString widestRollText =
                    formatNumber(minRollDeg, 2).size() >= formatNumber(maxRollDeg, 2).size()
                    ? formatNumber(minRollDeg, 2)
                    : formatNumber(maxRollDeg, 2);
            const QString widestPitchText =
                    formatNumber(minPitchDeg, 2).size() >= formatNumber(maxPitchDeg, 2).size()
                    ? formatNumber(minPitchDeg, 2)
                    : formatNumber(maxPitchDeg, 2);
            const QString widestYawText =
                    formatNumber(minYawDeg, 2).size() >= formatNumber(maxYawDeg, 2).size()
                    ? formatNumber(minYawDeg, 2)
                    : formatNumber(maxYawDeg, 2);
            expandLabelMinimumWidth(m_poseLabel,
                                    &m_poseMinWidth,
                                    QStringLiteral("姿态 Roll=%1 Pitch=%2 Yaw=%3")
                                            .arg(widestRollText, widestPitchText, widestYawText));
        }
    }

    rebuildPlotDataCache();
    pause();
    setControlsEnabled(true);
    if (m_poseSimulationView) {
        m_poseSimulationView->setSamples(m_samples, m_poseSceneGeometry);
    }
    initializePlotCarousels();
    m_timelineSlider->setRange(0, qMax(0, m_samples.size() - 1));
    updateJumpTimeRange();
    updateSummary(result, filePath);
    updateUiForIndex(0);
    updatePlotCarouselTimerState();
}

void MainWindow::updateUiForIndex(int index)
{
    if (m_samples.isEmpty()) {
        return;
    }

    index = qBound(0, index, m_samples.size() - 1);
    m_currentIndex = index;

    if (m_timelineSlider->value() != index) {
        m_timelineSlider->blockSignals(true);
        m_timelineSlider->setValue(index);
        m_timelineSlider->blockSignals(false);
    }

    const ReplaySample &sample = m_samples.at(index);
    const QString currentTimeText = QStringLiteral("当前时刻: %1").arg(formatTimestamp(sample));
    m_currentTimeLabel->setText(currentTimeText);
    expandLabelMinimumWidth(m_currentTimeLabel, &m_currentTimeMinWidth, currentTimeText);
    if (sample.poseValid) {
        const QString poseText = QStringLiteral("姿态 Roll=%1 Pitch=%2 Yaw=%3")
                .arg(formatNumber(sample.rollDeg, 2))
                .arg(formatNumber(sample.pitchDeg, 2))
                .arg(formatNumber(sample.yawDeg, 2));
        m_poseLabel->setText(poseText);
        expandLabelMinimumWidth(m_poseLabel, &m_poseMinWidth, poseText);
    } else {
        const QString poseText = QStringLiteral("姿态: 当前时刻不可用");
        m_poseLabel->setText(poseText);
        expandLabelMinimumWidth(m_poseLabel, &m_poseMinWidth, poseText);
    }

    syncJumpTimeControls(sample.playbackMs);

    if (isReplayTabActive()) {
        refreshReplayPageForCurrentSample();
    }
    if (isAccelerationTabActive() && m_accelerationVisualizationView) {
        m_accelerationVisualizationView->setSample(sample);
    }
    if (isPoseTabActive() && m_poseSimulationView) {
        m_poseSimulationView->setSample(sample, index);
    }

    scheduleFrameCostMeasurement();
}

void MainWindow::resetUiState()
{
    pause();
    m_samples.clear();
    m_poseSceneGeometry = PoseSceneGeometry();
    m_sampleIntervalMs = 5;
    m_sampleIntervalSeconds = 0.005;
    m_frameCostWarningThresholdMs = 5.0;
    if (m_playbackTimer) {
        m_playbackTimer->setInterval(static_cast<int>(m_sampleIntervalMs));
    }
    if (m_jumpMillisecondSpin) {
        m_jumpMillisecondSpin->setSingleStep(static_cast<int>(m_sampleIntervalMs));
    }
    clearPlotDataCache();
    m_currentIndex = -1;
    resetInfoCarouselState();
    m_currentTimeLabel->setText(QStringLiteral("当前时刻: --"));
    m_poseLabel->setText(QStringLiteral("姿态: --"));
    updateFrameCostLabel();
    m_timelineSlider->setRange(0, 0);
    m_timelineSlider->setValue(0);
    updateJumpTimeRange();
    syncJumpTimeControls(0);
    if (m_valueTable) {
        for (int row = 0; row < m_valueTable->rowCount(); ++row) {
            if (m_valueTable->item(row, 0)) {
                m_valueTable->item(row, 0)->setText(QString());
            }
            if (m_valueTable->item(row, 1)) {
                m_valueTable->item(row, 1)->setText(QString());
            }
        }
    }
    if (m_solutionTable) {
        for (int row = 0; row < m_solutionTable->rowCount(); ++row) {
            if (m_solutionTable->item(row, 0)) {
                m_solutionTable->item(row, 0)->setText(QString());
            }
            if (m_solutionTable->item(row, 1)) {
                m_solutionTable->item(row, 1)->setText(QString());
            }
        }
    }
    initializePlotCarousels();
    refreshPlots();
    if (m_accelerationVisualizationView) {
        m_accelerationVisualizationView->clearSample();
    }
    if (m_poseSimulationView) {
        m_poseSimulationView->clearSample();
    }
    setControlsEnabled(false);
    updatePlotCarouselTimerState();
}

QStringList MainWindow::buildTopInfoItems(const QString &filePath,
                                          const QString &summaryItem,
                                          const QString &warningMessage) const
{
    QStringList items;
    if (!filePath.isEmpty()) {
        items << QStringLiteral("传感器文件: %1").arg(filePath);
    }
    if (m_hasMileageCorrectionTable && !m_currentMileageCorrectionFilePath.isEmpty()) {
        items << QStringLiteral("里程校正文件: %1").arg(m_currentMileageCorrectionFilePath);
    }
    items.append(buildStatusInfoItems(summaryItem, warningMessage));
    return items;
}

QStringList MainWindow::buildStatusInfoItems(const QString &summaryItem,
                                             const QString &warningMessage) const
{
    QStringList items;
    if (!summaryItem.isEmpty()) {
        items << summaryItem;
    }

    const QStringList warningItems = warningMessage.split(QStringLiteral("；"), Qt::SkipEmptyParts);
    for (const QString &warningItem : warningItems) {
        const QString trimmedItem = warningItem.trimmed();
        if (!trimmedItem.isEmpty()) {
            items << trimmedItem;
        }
    }
    return items;
}

void MainWindow::updateTopInfoDisplay()
{
    if (!m_topInfoLabel) {
        return;
    }

    const QString text = m_topInfoItems.isEmpty()
            ? QStringLiteral("未加载传感器文件")
            : m_topInfoItems.at(qBound(0, m_topInfoIndex, m_topInfoItems.size() - 1));
    m_topInfoLabel->setText(text);
    m_topInfoLabel->setToolTip(text);
    expandLabelMinimumWidth(m_topInfoLabel, &m_topInfoMinWidth, text);
}

void MainWindow::updateStatusInfoDisplay()
{
    if (!m_statusCarouselLabel) {
        return;
    }

    const QString text = m_statusInfoItems.isEmpty()
            ? QStringLiteral("等待加载传感器文件")
            : m_statusInfoItems.at(qBound(0, m_statusInfoIndex, m_statusInfoItems.size() - 1));
    m_statusCarouselLabel->setText(text);
    m_statusCarouselLabel->setToolTip(text);
    expandLabelMinimumWidth(m_statusCarouselLabel, &m_statusInfoMinWidth, text);
}

void MainWindow::updateInfoCarouselTimerState()
{
    if (!m_infoCarouselTimer) {
        return;
    }

    const bool shouldRun = m_topInfoItems.size() > 1 || m_statusInfoItems.size() > 1;
    if (shouldRun) {
        if (!m_infoCarouselTimer->isActive()) {
            m_infoCarouselTimer->start();
        }
    } else if (m_infoCarouselTimer->isActive()) {
        m_infoCarouselTimer->stop();
    }
}

void MainWindow::resetInfoCarouselState()
{
    m_topInfoItems = QStringList() << QStringLiteral("未加载传感器文件");
    if (m_hasMileageCorrectionTable && !m_currentMileageCorrectionFilePath.isEmpty()) {
        m_topInfoItems << QStringLiteral("里程校正文件: %1").arg(m_currentMileageCorrectionFilePath);
    }
    m_statusInfoItems = QStringList() << QStringLiteral("等待加载传感器文件");
    if (m_hasMileageCorrectionTable && !m_currentMileageCorrectionFilePath.isEmpty()) {
        m_statusInfoItems << QStringLiteral("里程校正文件已加载，等待加载传感器文件");
    }
    m_topInfoIndex = 0;
    m_statusInfoIndex = 0;
    m_topInfoHovered = false;
    m_statusInfoHovered = false;
    updateTopInfoDisplay();
    updateStatusInfoDisplay();
    if (statusBar()) {
        statusBar()->clearMessage();
    }
    updateInfoCarouselTimerState();
}

void MainWindow::expandLabelMinimumWidth(QLabel *label, int *cachedWidth, const QString &text)
{
    if (!label || !cachedWidth) {
        return;
    }

    const QFontMetrics metrics(label->font());
    const int requiredWidth = metrics.horizontalAdvance(text) + label->margin() * 2 + 24;
    if (requiredWidth > *cachedWidth) {
        *cachedWidth = requiredWidth;
        label->setMinimumWidth(*cachedWidth);
    }
}

void MainWindow::updateSummary(const ParseResult &result, const QString &filePath)
{
    setWindowTitle(QStringLiteral("原始数据回放 - %1").arg(filePath));
    Q_UNUSED(result)
}

void MainWindow::advanceInfoCarousel()
{
    bool topChanged = false;
    if (!m_topInfoHovered && m_topInfoItems.size() > 1) {
        m_topInfoIndex = (m_topInfoIndex + 1) % m_topInfoItems.size();
        topChanged = true;
    }

    bool statusChanged = false;
    if (!m_statusInfoHovered && m_statusInfoItems.size() > 1) {
        m_statusInfoIndex = (m_statusInfoIndex + 1) % m_statusInfoItems.size();
        statusChanged = true;
    }

    if (topChanged) {
        updateTopInfoDisplay();
    }
    if (statusChanged) {
        updateStatusInfoDisplay();
    }
}

void MainWindow::refreshPlots()
{
    if (m_samples.isEmpty()) {
        if (m_accelPlot) {
            m_accelPlot->clearGraphs();
            m_accelPlot->replot(QCustomPlot::rpQueuedReplot);
        }
        if (m_gyroPlot) {
            m_gyroPlot->clearGraphs();
            m_gyroPlot->replot(QCustomPlot::rpQueuedReplot);
        }
        if (m_posePlot) {
            m_posePlot->clearGraphs();
            m_posePlot->replot(QCustomPlot::rpQueuedReplot);
        }
        if (m_noisePlot) {
            m_noisePlot->clearGraphs();
            m_noisePlot->replot(QCustomPlot::rpQueuedReplot);
        }
        if (m_speedPlot) {
            m_speedPlot->clearGraphs();
            m_speedPlot->replot(QCustomPlot::rpQueuedReplot);
        }
        return;
    }

    if (m_suspendReplayPlotRendering) {
        return;
    }

    refreshSinglePlot(m_accelPlot,
                      m_accelDisplayCombo,
                      m_accelViewModeCombo,
                      QVector<QVector<double> >() << m_accelXData << m_accelYData << m_accelZData,
                      QVector<QVector<QVector<int> > >() << m_accelXHistorySampledIndices << m_accelYHistorySampledIndices << m_accelZHistorySampledIndices,
                      QVector<QString>() << QStringLiteral("Accel X") << QStringLiteral("Accel Y") << QStringLiteral("Accel Z"),
                      QStringLiteral("加速度 (g)"));
    refreshSinglePlot(m_gyroPlot,
                      m_gyroDisplayCombo,
                      m_gyroViewModeCombo,
                      QVector<QVector<double> >() << m_gyroXData << m_gyroYData << m_gyroZData,
                      QVector<QVector<QVector<int> > >() << m_gyroXHistorySampledIndices << m_gyroYHistorySampledIndices << m_gyroZHistorySampledIndices,
                      QVector<QString>() << QStringLiteral("Gyro X") << QStringLiteral("Gyro Y") << QStringLiteral("Gyro Z"),
                      QStringLiteral("角速度 (deg/s)"));
    refreshSinglePlot(m_posePlot,
                      m_poseDisplayCombo,
                      m_poseViewModeCombo,
                      QVector<QVector<double> >() << m_rollData << m_pitchData << m_yawData,
                      QVector<QVector<QVector<int> > >() << m_rollHistorySampledIndices << m_pitchHistorySampledIndices << m_yawHistorySampledIndices,
                      QVector<QString>() << QStringLiteral("Roll") << QStringLiteral("Pitch") << QStringLiteral("Yaw"),
                      QStringLiteral("姿态角 (deg)"));
    refreshNoisePlot();
    refreshSpeedPlot();
}

void MainWindow::refreshNoisePlot()
{
    refreshSinglePlot(m_noisePlot,
                      nullptr,
                      m_noiseViewModeCombo,
                      QVector<QVector<double> >() << m_noiseData,
                      QVector<QVector<QVector<int> > >() << m_noiseHistorySampledIndices,
                      QVector<QString>() << QStringLiteral("Noise"),
                      QStringLiteral("噪声 (dBA)"));
}

void MainWindow::refreshSpeedPlot()
{
    refreshSinglePlot(m_speedPlot,
                      nullptr,
                      m_speedViewModeCombo,
                      QVector<QVector<double> >() << m_speedZData,
                      QVector<QVector<QVector<int> > >() << m_speedZHistorySampledIndices,
                      QVector<QString>() << QStringLiteral("Speed Z"),
                      QStringLiteral("解算速度 Z (g*s)"));
}

void MainWindow::configurePlot(QCustomPlot *plot)
{
    if (!plot) {
        return;
    }

    plot->setInteractions(QCP::iNone);
    plot->setNoAntialiasingOnDrag(true);
    plot->legend->setVisible(false);
    plot->legend->setSelectableParts(QCPLegend::spNone);
    plot->xAxis->grid()->setVisible(true);
    plot->yAxis->grid()->setVisible(true);
}

bool MainWindow::isReplayTabActive() const
{
    return m_mainTabs && m_mainTabs->currentIndex() == kReplayTabIndex;
}

bool MainWindow::isAccelerationTabActive() const
{
    return m_mainTabs && m_mainTabs->currentIndex() == kAccelerationTabIndex;
}

bool MainWindow::isPoseTabActive() const
{
    return m_mainTabs && m_mainTabs->currentIndex() == kPoseTabIndex;
}

void MainWindow::refreshReplayPageForCurrentSample()
{
    if (m_samples.isEmpty() || m_currentIndex < 0 || m_currentIndex >= m_samples.size()) {
        return;
    }

    const ReplaySample &sample = m_samples.at(m_currentIndex);
    const QStringList keys = {
        QStringLiteral("序号"),
        QStringLiteral("回放时间 (ms)"),
        QStringLiteral("GPS 时间"),
        QStringLiteral("经纬高"),
        QStringLiteral("速度 Vx/Vy/Vz"),
        QStringLiteral("加速度 X/Y/Z"),
        QStringLiteral("角速度 X/Y/Z"),
        QStringLiteral("噪声"),
        QStringLiteral("气压 / 温度"),
        QStringLiteral("状态标志")
    };

    const QStringList values = {
        QString::number(sample.sequence),
        QString::number(sample.playbackMs),
        formatTimestamp(sample),
        sample.hasGps ? QStringLiteral("%1, %2, %3 m")
                        .arg(formatNumber(sample.latitudeDeg, 7))
                        .arg(formatNumber(sample.longitudeDeg, 7))
                        .arg(formatNumber(sample.altitudeM, 2))
                      : QStringLiteral("无"),
        sample.hasGps ? QStringLiteral("%1 / %2 / %3 m/s")
                        .arg(formatNumber(sample.vxMps, 2))
                        .arg(formatNumber(sample.vyMps, 2))
                        .arg(formatNumber(sample.vzMps, 2))
                      : QStringLiteral("无"),
        sample.hasImu ? QStringLiteral("%1 / %2 / %3 g")
                        .arg(formatNumber(sample.accelXG, 3))
                        .arg(formatNumber(sample.accelYG, 3))
                        .arg(formatNumber(sample.accelZG, 3))
                      : QStringLiteral("无"),
        sample.hasImu ? QStringLiteral("%1 / %2 / %3 deg/s")
                        .arg(formatNumber(sample.gyroXDeg, 2))
                        .arg(formatNumber(sample.gyroYDeg, 2))
                        .arg(formatNumber(sample.gyroZDeg, 2))
                      : QStringLiteral("无"),
        sample.hasGps ? QStringLiteral("%1 dBA").arg(formatNumber(sample.noiseDbA, 1))
                      : QStringLiteral("无"),
        sample.hasImu ? QStringLiteral("%1 hPa / %2 C")
                        .arg(formatNumber(sample.pressureHpa, 2))
                        .arg(formatNumber(sample.temperatureC, 1))
                      : QStringLiteral("无"),
        sample.hasImu ? QStringLiteral("0x%1").arg(sample.statusFlag, 2, 16, QLatin1Char('0')).toUpper()
                      : QStringLiteral("无")
    };

    for (int i = 0; i < keys.size(); ++i) {
        if (m_valueTable->item(i, 0)) {
            m_valueTable->item(i, 0)->setText(keys.at(i));
        }
        if (m_valueTable->item(i, 1)) {
            m_valueTable->item(i, 1)->setText(values.at(i));
        }
    }

    const QStringList solutionKeys = {
        QStringLiteral("解算速度 (修正后 Accel Z 积分)"),
        QStringLiteral("当前姿态 Roll/Pitch/Yaw")
    };

    const QStringList solutionValues = {
        QStringLiteral("%1 g*s").arg(formatNumber(sample.derivedSpeedZGs, 3)),
        sample.poseValid ? QStringLiteral("%1 / %2 / %3 deg")
                           .arg(formatNumber(sample.rollDeg, 2))
                           .arg(formatNumber(sample.pitchDeg, 2))
                           .arg(formatNumber(sample.yawDeg, 2))
                         : QStringLiteral("无")
    };

    if (m_solutionTable) {
        for (int i = 0; i < solutionKeys.size(); ++i) {
            if (m_solutionTable->item(i, 0)) {
                m_solutionTable->item(i, 0)->setText(solutionKeys.at(i));
            }
            if (m_solutionTable->item(i, 1)) {
                m_solutionTable->item(i, 1)->setText(solutionValues.at(i));
                if (i == 0 && qAbs(sample.derivedSpeedZGs) > kSpeedAlertThresholdGs) {
                    m_solutionTable->item(i, 1)->setForeground(QBrush(QColor(255, 96, 96)));
                } else {
                    m_solutionTable->item(i, 1)->setForeground(QBrush());
                }
            }
        }
    }

    if (!isStaticOverviewMode(m_accelViewModeCombo)) {
        refreshSinglePlot(m_accelPlot,
                          m_accelDisplayCombo,
                          m_accelViewModeCombo,
                          QVector<QVector<double> >() << m_accelXData << m_accelYData << m_accelZData,
                          QVector<QVector<QVector<int> > >() << m_accelXHistorySampledIndices << m_accelYHistorySampledIndices << m_accelZHistorySampledIndices,
                          QVector<QString>() << QStringLiteral("Accel X") << QStringLiteral("Accel Y") << QStringLiteral("Accel Z"),
                          QStringLiteral("加速度 (g)"));
    }
    if (!isStaticOverviewMode(m_gyroViewModeCombo)) {
        refreshSinglePlot(m_gyroPlot,
                          m_gyroDisplayCombo,
                          m_gyroViewModeCombo,
                          QVector<QVector<double> >() << m_gyroXData << m_gyroYData << m_gyroZData,
                          QVector<QVector<QVector<int> > >() << m_gyroXHistorySampledIndices << m_gyroYHistorySampledIndices << m_gyroZHistorySampledIndices,
                          QVector<QString>() << QStringLiteral("Gyro X") << QStringLiteral("Gyro Y") << QStringLiteral("Gyro Z"),
                          QStringLiteral("角速度 (deg/s)"));
    }
    if (!isStaticOverviewMode(m_poseViewModeCombo)) {
        refreshSinglePlot(m_posePlot,
                          m_poseDisplayCombo,
                          m_poseViewModeCombo,
                          QVector<QVector<double> >() << m_rollData << m_pitchData << m_yawData,
                          QVector<QVector<QVector<int> > >() << m_rollHistorySampledIndices << m_pitchHistorySampledIndices << m_yawHistorySampledIndices,
                          QVector<QString>() << QStringLiteral("Roll") << QStringLiteral("Pitch") << QStringLiteral("Yaw"),
                          QStringLiteral("姿态角 (deg)"));
    }
    if (!isStaticOverviewMode(m_noiseViewModeCombo)) {
        refreshNoisePlot();
    }
    if (!isStaticOverviewMode(m_speedViewModeCombo)) {
        refreshSpeedPlot();
    }
}

void MainWindow::refreshCurrentVisiblePage()
{
    if (m_samples.isEmpty() || m_currentIndex < 0 || m_currentIndex >= m_samples.size()) {
        return;
    }

    if (isReplayTabActive()) {
        refreshReplayPageForCurrentSample();
        return;
    }

    const ReplaySample &sample = m_samples.at(m_currentIndex);
    if (isAccelerationTabActive()) {
        if (m_accelerationVisualizationView) {
            m_accelerationVisualizationView->setSample(sample);
        }
        return;
    }

    if (isPoseTabActive() && m_poseSimulationView) {
        m_poseSimulationView->setSample(sample, m_currentIndex);
        return;
    }
}

void MainWindow::rebuildPlotDataCache()
{
    clearPlotDataCache();

    const int sampleCount = m_samples.size();
    m_plotTimeData.reserve(sampleCount);
    m_accelXData.reserve(sampleCount);
    m_accelYData.reserve(sampleCount);
    m_accelZData.reserve(sampleCount);
    m_gyroXData.reserve(sampleCount);
    m_gyroYData.reserve(sampleCount);
    m_gyroZData.reserve(sampleCount);
    m_rollData.reserve(sampleCount);
    m_pitchData.reserve(sampleCount);
    m_yawData.reserve(sampleCount);
    m_noiseData.reserve(sampleCount);
    m_speedZData.reserve(sampleCount);

    for (const ReplaySample &sample : qAsConst(m_samples)) {
        m_plotTimeData.push_back(static_cast<double>(sample.playbackMs));
        m_accelXData.push_back(sample.accelXG);
        m_accelYData.push_back(sample.accelYG);
        m_accelZData.push_back(sample.accelZG);
        m_gyroXData.push_back(sample.gyroXDeg);
        m_gyroYData.push_back(sample.gyroYDeg);
        m_gyroZData.push_back(sample.gyroZDeg);
        m_rollData.push_back(sample.rollDeg);
        m_pitchData.push_back(sample.pitchDeg);
        m_yawData.push_back(sample.yawDeg);
        m_noiseData.push_back(sample.noiseDbA);
        m_speedZData.push_back(sample.derivedSpeedZGs);
    }

    if (m_samples.isEmpty()) {
        return;
    }

    const int maxHalfSecondBlock = qMax(0, static_cast<int>(qMax<qint64>(0, m_samples.constLast().playbackMs)
                                                             / kDynamicFollowRawSegmentMs));
    m_halfSecondStartIndices.fill(-1, maxHalfSecondBlock + 1);
    m_halfSecondEndIndices.fill(-1, maxHalfSecondBlock + 1);
    for (int index = 0; index < m_samples.size(); ++index) {
        const int halfSecondBlock = qMax(0, static_cast<int>(qMax<qint64>(0, m_samples.at(index).playbackMs)
                                                             / kDynamicFollowRawSegmentMs));
        if (m_halfSecondStartIndices.at(halfSecondBlock) < 0) {
            m_halfSecondStartIndices[halfSecondBlock] = index;
        }
        m_halfSecondEndIndices[halfSecondBlock] = index;
    }

    m_accelXHistorySampledIndices =
            buildHistoryBucketExtremaIndexCache(m_accelXData, m_halfSecondStartIndices, m_halfSecondEndIndices, kDynamicFollowHistoryBucketSize);
    m_accelYHistorySampledIndices =
            buildHistoryBucketExtremaIndexCache(m_accelYData, m_halfSecondStartIndices, m_halfSecondEndIndices, kDynamicFollowHistoryBucketSize);
    m_accelZHistorySampledIndices =
            buildHistoryBucketExtremaIndexCache(m_accelZData, m_halfSecondStartIndices, m_halfSecondEndIndices, kDynamicFollowHistoryBucketSize);
    m_gyroXHistorySampledIndices =
            buildHistoryBucketExtremaIndexCache(m_gyroXData, m_halfSecondStartIndices, m_halfSecondEndIndices, kDynamicFollowHistoryBucketSize);
    m_gyroYHistorySampledIndices =
            buildHistoryBucketExtremaIndexCache(m_gyroYData, m_halfSecondStartIndices, m_halfSecondEndIndices, kDynamicFollowHistoryBucketSize);
    m_gyroZHistorySampledIndices =
            buildHistoryBucketExtremaIndexCache(m_gyroZData, m_halfSecondStartIndices, m_halfSecondEndIndices, kDynamicFollowHistoryBucketSize);
    m_rollHistorySampledIndices =
            buildHistoryBucketExtremaIndexCache(m_rollData, m_halfSecondStartIndices, m_halfSecondEndIndices, kDynamicFollowHistoryBucketSize);
    m_pitchHistorySampledIndices =
            buildHistoryBucketExtremaIndexCache(m_pitchData, m_halfSecondStartIndices, m_halfSecondEndIndices, kDynamicFollowHistoryBucketSize);
    m_yawHistorySampledIndices =
            buildHistoryBucketExtremaIndexCache(m_yawData, m_halfSecondStartIndices, m_halfSecondEndIndices, kDynamicFollowHistoryBucketSize);
    m_noiseHistorySampledIndices =
            buildHistoryBucketExtremaIndexCache(m_noiseData, m_halfSecondStartIndices, m_halfSecondEndIndices, kDynamicFollowHistoryBucketSize);
    m_speedZHistorySampledIndices =
            buildHistoryBucketExtremaIndexCache(m_speedZData, m_halfSecondStartIndices, m_halfSecondEndIndices, kDynamicFollowHistoryBucketSize);
}

void MainWindow::clearPlotDataCache()
{
    m_plotTimeData.clear();
    m_accelXData.clear();
    m_accelYData.clear();
    m_accelZData.clear();
    m_gyroXData.clear();
    m_gyroYData.clear();
    m_gyroZData.clear();
    m_rollData.clear();
    m_pitchData.clear();
    m_yawData.clear();
    m_noiseData.clear();
    m_speedZData.clear();
    m_halfSecondStartIndices.clear();
    m_halfSecondEndIndices.clear();
    m_accelXHistorySampledIndices.clear();
    m_accelYHistorySampledIndices.clear();
    m_accelZHistorySampledIndices.clear();
    m_gyroXHistorySampledIndices.clear();
    m_gyroYHistorySampledIndices.clear();
    m_gyroZHistorySampledIndices.clear();
    m_rollHistorySampledIndices.clear();
    m_pitchHistorySampledIndices.clear();
    m_yawHistorySampledIndices.clear();
    m_noiseHistorySampledIndices.clear();
    m_speedZHistorySampledIndices.clear();
    m_accelOverviewCache = StaticOverviewSeriesCache();
    m_gyroOverviewCache = StaticOverviewSeriesCache();
    m_poseOverviewCache = StaticOverviewSeriesCache();
    m_noiseOverviewCache = StaticOverviewSeriesCache();
    m_speedOverviewCache = StaticOverviewSeriesCache();
}

void MainWindow::initializePlotCarousels()
{
    setPlotCarouselEnabled(&m_accelCarouselContext, true);
    setPlotCarouselEnabled(&m_gyroCarouselContext, true);
    setPlotCarouselEnabled(&m_poseCarouselContext, true);
    setPlotCarouselEnabled(&m_noiseCarouselContext, true);
    setPlotCarouselEnabled(&m_speedCarouselContext, true);
}

void MainWindow::updatePlotCarouselTimerState()
{
    if (!m_plotCarouselTimer) {
        return;
    }

    const bool anyCarouselEnabled =
            (m_accelCarouselRadio && m_accelCarouselRadio->isChecked())
            || (m_gyroCarouselRadio && m_gyroCarouselRadio->isChecked())
            || (m_poseCarouselRadio && m_poseCarouselRadio->isChecked())
            || (m_noiseCarouselRadio && m_noiseCarouselRadio->isChecked())
            || (m_speedCarouselRadio && m_speedCarouselRadio->isChecked());
    const bool shouldRun = !m_samples.isEmpty() && isReplayTabActive() && anyCarouselEnabled;
    if (shouldRun) {
        if (!m_plotCarouselTimer->isActive()) {
            m_plotCarouselTimer->start();
        }
    } else if (m_plotCarouselTimer->isActive()) {
        m_plotCarouselTimer->stop();
    }
}

void MainWindow::advancePlotCarousels()
{
    if (m_samples.isEmpty() || !isReplayTabActive()) {
        updatePlotCarouselTimerState();
        return;
    }

    bool hasAdvanced = false;
    if (m_accelCarouselRadio && m_accelCarouselRadio->isChecked()) {
        advancePlotCarousel(&m_accelCarouselContext);
        hasAdvanced = true;
    }
    if (m_gyroCarouselRadio && m_gyroCarouselRadio->isChecked()) {
        advancePlotCarousel(&m_gyroCarouselContext);
        hasAdvanced = true;
    }
    if (m_poseCarouselRadio && m_poseCarouselRadio->isChecked()) {
        advancePlotCarousel(&m_poseCarouselContext);
        hasAdvanced = true;
    }
    if (m_noiseCarouselRadio && m_noiseCarouselRadio->isChecked()) {
        advancePlotCarousel(&m_noiseCarouselContext);
        hasAdvanced = true;
    }
    if (m_speedCarouselRadio && m_speedCarouselRadio->isChecked()) {
        advancePlotCarousel(&m_speedCarouselContext);
        hasAdvanced = true;
    }

    if (hasAdvanced && isReplayTabActive()) {
        refreshPlots();
    }
}

void MainWindow::advancePlotCarousel(PlotCarouselContext *context)
{
    if (!context) {
        return;
    }

    const int combinationCount = plotCarouselCombinationCount(context);
    if (combinationCount <= 0) {
        return;
    }

    const int currentCombination = currentPlotCarouselCombinationIndex(context);
    const int nextCombination = (currentCombination + 1) % combinationCount;
    setPlotCarouselCombination(context, nextCombination);
}

void MainWindow::setPlotCarouselCombination(PlotCarouselContext *context, int combinationIndex)
{
    if (!context || !context->viewModeCombo) {
        return;
    }

    const int combinationCount = plotCarouselCombinationCount(context);
    if (combinationCount <= 0) {
        return;
    }

    const int displayCount = context->displayCombo ? qMax(1, context->displayCombo->count()) : 1;
    const int normalizedIndex = (combinationIndex % combinationCount + combinationCount) % combinationCount;
    const int viewIndex = normalizedIndex / displayCount;
    const int displayIndex = normalizedIndex % displayCount;
    const QSignalBlocker viewBlocker(context->viewModeCombo);
    m_plotCarouselProgrammaticChange = true;
    context->viewModeCombo->setCurrentIndex(viewIndex);
    if (context->displayCombo) {
        const QSignalBlocker displayBlocker(context->displayCombo);
        context->displayCombo->setCurrentIndex(displayIndex);
    }
    m_plotCarouselProgrammaticChange = false;
}

void MainWindow::setPlotCarouselEnabled(PlotCarouselContext *context, bool enabled)
{
    if (!context || !context->carouselRadio) {
        return;
    }

    const QSignalBlocker blocker(context->carouselRadio);
    context->carouselRadio->setChecked(enabled);
    updatePlotCarouselTimerState();
}

int MainWindow::currentPlotCarouselCombinationIndex(const PlotCarouselContext *context) const
{
    if (!context || !context->viewModeCombo) {
        return 0;
    }

    const int displayCount = context->displayCombo ? qMax(1, context->displayCombo->count()) : 1;
    const int displayIndex = context->displayCombo ? qMax(0, context->displayCombo->currentIndex()) : 0;
    return context->viewModeCombo->currentIndex() * displayCount + displayIndex;
}

int MainWindow::plotCarouselCombinationCount(const PlotCarouselContext *context) const
{
    if (!context || !context->viewModeCombo) {
        return 0;
    }

    const int displayCount = context->displayCombo ? qMax(1, context->displayCombo->count()) : 1;
    return context->viewModeCombo->count() * displayCount;
}

MainWindow::PlotCarouselContext *MainWindow::plotCarouselContextForObject(const QObject *object)
{
    if (!object) {
        return nullptr;
    }

    const PlotCarouselContext *contexts[] = {
        &m_accelCarouselContext,
        &m_gyroCarouselContext,
        &m_poseCarouselContext,
        &m_noiseCarouselContext,
        &m_speedCarouselContext
    };
    for (const PlotCarouselContext *context : contexts) {
        if (object == context->viewModeCombo || object == context->displayCombo || object == context->carouselRadio) {
            return const_cast<PlotCarouselContext *>(context);
        }
    }
    return nullptr;
}

void MainWindow::refreshSinglePlot(QCustomPlot *plot,
                                   const QComboBox *seriesComboBox,
                                   const QComboBox *viewModeComboBox,
                                   const QVector<QVector<double> > &series,
                                   const QVector<QVector<QVector<int> > > &sampledHistoryIndexCaches,
                                   const QVector<QString> &names,
                                   const QString &yLabel)
{
    if (!plot) {
        return;
    }

    QWidget *plotBox = plot->parentWidget();
    if (!shouldRenderReplayPlot(plotBox, plot)) {
        return;
    }

    plot->clearGraphs();
    const QVector<int> visibleIndices = selectedSeriesIndices(seriesComboBox, series.size());
    if (isStaticOverviewMode(viewModeComboBox)) {
        buildStaticOverviewPlot(plot, m_plotTimeData, series, names, visibleIndices, yLabel);
    } else {
        buildDynamicFollowPlot(plot, m_plotTimeData, series, sampledHistoryIndexCaches, names, visibleIndices, yLabel, m_currentIndex);
    }
    plot->replot(QCustomPlot::rpQueuedReplot);
}

void MainWindow::buildStaticOverviewPlot(QCustomPlot *plot,
                                         const QVector<double> &xData,
                                         const QVector<QVector<double> > &series,
                                         const QVector<QString> &names,
                                         const QVector<int> &visibleIndices,
                                         const QString &yLabel)
{
    if (!plot) {
        return;
    }

    StaticOverviewSeriesCache *cache = staticOverviewCacheForPlot(plot);
    const int targetWidth = staticOverviewTargetWidth(plot);
    if (cache && (cache->targetWidth != targetWidth || cache->downsampledSeries.size() != series.size())) {
        cache->targetWidth = targetWidth;
        cache->downsampledSeries.clear();
        cache->downsampledSeries.reserve(series.size());
        for (const QVector<double> &yData : series) {
            cache->downsampledSeries.push_back(buildDownsampledStaticOverviewSeries(xData, yData, targetWidth));
        }
    }

    const bool showLegend = visibleIndices.size() > 1;
    plot->legend->setVisible(showLegend);

    for (int visibleOrder = 0; visibleOrder < visibleIndices.size(); ++visibleOrder) {
        const int seriesIndex = visibleIndices.at(visibleOrder);
        const QPair<QVector<double>, QVector<double> > plotData =
                (cache && seriesIndex >= 0 && seriesIndex < cache->downsampledSeries.size())
                ? cache->downsampledSeries.at(seriesIndex)
                : buildDownsampledStaticOverviewSeries(xData, series.at(seriesIndex), targetWidth);
        plot->addGraph();
        plot->graph(visibleOrder)->setName(names.at(seriesIndex));
        plot->graph(visibleOrder)->setPen(QPen(plotSeriesColor(seriesIndex), 1.4));
        plot->graph(visibleOrder)->setAdaptiveSampling(false);
        plot->graph(visibleOrder)->setData(plotData.first, plotData.second);
        if (!showLegend) {
            plot->graph(visibleOrder)->removeFromLegend();
        }
    }

    plot->xAxis->setTicker(QSharedPointer<QCPAxisTicker>(new QCPAxisTicker));
    plot->xAxis->setLabel(QStringLiteral("回放时间 (ms)"));
    plot->xAxis->setNumberFormat(QStringLiteral("f"));
    plot->xAxis->setNumberPrecision(0);
    plot->yAxis->setLabel(yLabel);
    if (!xData.isEmpty()) {
        plot->xAxis->setRange(xData.first(), xData.last());
    }
    if (plot->graphCount() > 0) {
        plot->rescaleAxes();
    }
}

int MainWindow::staticOverviewTargetWidth(const QCustomPlot *plot) const
{
    if (!plot) {
        return 1;
    }

    const QCPAxisRect *axisRect = plot->axisRect();
    if (axisRect && axisRect->width() > 0) {
        return axisRect->width();
    }

    const int viewportWidth = plot->viewport().width();
    if (viewportWidth > 0) {
        return viewportWidth;
    }

    return qMax(1, plot->width());
}

int MainWindow::downsamplePointBudgetForWidth(int targetWidth) const
{
    const int safeWidth = qMax(1, targetWidth);
    return qMax(kStaticOverviewMinimumPointBudget, safeWidth * kStaticOverviewPointsPerPixel);
}

QPair<QVector<double>, QVector<double> > MainWindow::buildDownsampledStaticOverviewSeries(const QVector<double> &xData,
                                                                                           const QVector<double> &yData,
                                                                                           int targetWidth) const
{
    QPair<QVector<double>, QVector<double> > result;
    if (xData.isEmpty() || yData.isEmpty() || xData.size() != yData.size()) {
        return result;
    }

    const int pointBudget = downsamplePointBudgetForWidth(targetWidth);
    const int sampleCount = xData.size();
    if (sampleCount <= pointBudget) {
        result.first = xData;
        result.second = yData;
        return result;
    }

    const int bucketCount = qMax(1, pointBudget / kStaticOverviewMaxBucketSamples);
    const double xMin = xData.first();
    const double xMax = xData.last();
    const double xRange = xMax - xMin;

    QVector<BucketPointRange> buckets(bucketCount);
    for (int index = 0; index < sampleCount; ++index) {
        int bucketIndex = 0;
        if (xRange > 0.0) {
            const double normalized = (xData.at(index) - xMin) / xRange;
            bucketIndex = qBound(0, static_cast<int>(normalized * bucketCount), bucketCount - 1);
        } else {
            bucketIndex = qMin(bucketCount - 1, (index * bucketCount) / sampleCount);
        }

        BucketPointRange &bucket = buckets[bucketIndex];
        const double yValue = yData.at(index);
        if (!bucket.hasPoint) {
            bucket.hasPoint = true;
            bucket.firstIndex = index;
            bucket.lastIndex = index;
            bucket.minIndex = index;
            bucket.maxIndex = index;
            bucket.minY = yValue;
            bucket.maxY = yValue;
            continue;
        }

        bucket.lastIndex = index;
        if (yValue < bucket.minY) {
            bucket.minY = yValue;
            bucket.minIndex = index;
        }
        if (yValue > bucket.maxY) {
            bucket.maxY = yValue;
            bucket.maxIndex = index;
        }
    }

    result.first.reserve(pointBudget);
    result.second.reserve(pointBudget);
    for (const BucketPointRange &bucket : qAsConst(buckets)) {
        if (!bucket.hasPoint) {
            continue;
        }

        QVector<int> selectedIndices;
        selectedIndices.reserve(kStaticOverviewMaxBucketSamples);
        selectedIndices.push_back(bucket.firstIndex);
        selectedIndices.push_back(bucket.minIndex);
        selectedIndices.push_back(bucket.maxIndex);
        selectedIndices.push_back(bucket.lastIndex);
        std::sort(selectedIndices.begin(), selectedIndices.end());
        selectedIndices.erase(std::unique(selectedIndices.begin(), selectedIndices.end()), selectedIndices.end());

        for (int selectedIndex : qAsConst(selectedIndices)) {
            result.first.push_back(xData.at(selectedIndex));
            result.second.push_back(yData.at(selectedIndex));
        }
    }

    if (result.first.isEmpty()) {
        result.first = xData;
        result.second = yData;
    }
    return result;
}

MainWindow::StaticOverviewSeriesCache *MainWindow::staticOverviewCacheForPlot(const QCustomPlot *plot)
{
    if (plot == m_accelPlot) {
        return &m_accelOverviewCache;
    }
    if (plot == m_gyroPlot) {
        return &m_gyroOverviewCache;
    }
    if (plot == m_posePlot) {
        return &m_poseOverviewCache;
    }
    if (plot == m_noisePlot) {
        return &m_noiseOverviewCache;
    }
    if (plot == m_speedPlot) {
        return &m_speedOverviewCache;
    }
    return nullptr;
}

void MainWindow::buildDynamicFollowPlot(QCustomPlot *plot,
                                        const QVector<double> &xData,
                                        const QVector<QVector<double> > &series,
                                        const QVector<QVector<QVector<int> > > &sampledHistoryIndexCaches,
                                        const QVector<QString> &names,
                                        const QVector<int> &visibleIndices,
                                        const QString &yLabel,
                                        int currentIndex)
{
    if (!plot) {
        return;
    }

    const bool showLegend = visibleIndices.size() > 1;
    plot->legend->setVisible(showLegend);
    const int clampedIndex = xData.isEmpty() ? -1 : qBound(0, currentIndex, xData.size() - 1);
    const qint64 currentPlaybackMs = clampedIndex >= 0
            ? qMax<qint64>(0, static_cast<qint64>(xData.at(clampedIndex)))
            : 0;
    qint64 windowStartMs = 0;
    qint64 windowEndMs = kDynamicFollowWindowMs;
    if (currentPlaybackMs >= kDynamicFollowWindowMs) {
        const qint64 currentHalfSecondBlock = currentPlaybackMs / kDynamicFollowRawSegmentMs;
        const qint64 currentRawSegmentStartMs = currentHalfSecondBlock * kDynamicFollowRawSegmentMs;
        windowStartMs = qMax<qint64>(0, currentRawSegmentStartMs - kDynamicFollowSampledHistoryMs);
        windowEndMs = windowStartMs + kDynamicFollowWindowMs;
    }

    for (int visibleOrder = 0; visibleOrder < visibleIndices.size(); ++visibleOrder) {
        const int seriesIndex = visibleIndices.at(visibleOrder);
        const QPair<QVector<double>, QVector<double> > dynamicSeries =
                buildDynamicWindowSeries(xData,
                                         series.at(seriesIndex),
                                         sampledHistoryIndexCaches.at(seriesIndex),
                                         currentIndex);
        plot->addGraph();
        plot->graph(visibleOrder)->setName(names.at(seriesIndex));
        plot->graph(visibleOrder)->setPen(QPen(plotSeriesColor(seriesIndex), 1.6));
        plot->graph(visibleOrder)->setAdaptiveSampling(false);
        plot->graph(visibleOrder)->setData(dynamicSeries.first, dynamicSeries.second);
        if (!showLegend) {
            plot->graph(visibleOrder)->removeFromLegend();
        }
    }

    QSharedPointer<QCPAxisTickerFixed> secondTicker(new QCPAxisTickerFixed);
    secondTicker->setTickStep(static_cast<double>(kDynamicFollowRawSegmentMs));
    plot->xAxis->setTicker(secondTicker);
    plot->xAxis->setLabel(QStringLiteral("回放时间 (ms)"));
    plot->xAxis->setNumberFormat(QStringLiteral("f"));
    plot->xAxis->setNumberPrecision(0);
    plot->yAxis->setLabel(yLabel);
    if (plot->graphCount() > 0) {
        plot->rescaleAxes();
    }
    plot->xAxis->setRange(static_cast<double>(windowStartMs), static_cast<double>(windowEndMs));
}

QPair<QVector<double>, QVector<double> > MainWindow::buildDynamicWindowSeries(const QVector<double> &xData,
                                                                               const QVector<double> &yData,
                                                                              const QVector<QVector<int> > &sampledHistoryIndexCache,
                                                                               int currentIndex) const
{
    QPair<QVector<double>, QVector<double> > result;
    if (xData.isEmpty() || yData.isEmpty() || xData.size() != yData.size() || currentIndex < 0) {
        return result;
    }

    const int clampedIndex = qBound(0, currentIndex, xData.size() - 1);
    const qint64 currentPlaybackMs = qMax<qint64>(0, static_cast<qint64>(xData.at(clampedIndex)));
    const int currentHalfSecondBlock = static_cast<int>(currentPlaybackMs / kDynamicFollowRawSegmentMs);
    const int sampledHistoryBlockCount = static_cast<int>(kDynamicFollowSampledHistoryMs / kDynamicFollowRawSegmentMs);
    const int sampledHistoryStartBlock = qMax(0, currentHalfSecondBlock - sampledHistoryBlockCount);

    for (int halfSecondBlock = sampledHistoryStartBlock; halfSecondBlock < currentHalfSecondBlock; ++halfSecondBlock) {
        if (halfSecondBlock < 0 || halfSecondBlock >= m_halfSecondStartIndices.size()
                || halfSecondBlock >= m_halfSecondEndIndices.size()
                || halfSecondBlock >= sampledHistoryIndexCache.size()) {
            continue;
        }
        const QVector<int> &sampledIndices = sampledHistoryIndexCache.at(halfSecondBlock);
        for (int sampleIndex : sampledIndices) {
            if (sampleIndex < 0 || sampleIndex >= xData.size() || sampleIndex >= yData.size()) {
                continue;
            }
            result.first.push_back(xData.at(sampleIndex));
            result.second.push_back(yData.at(sampleIndex));
        }
    }

    if (currentHalfSecondBlock >= 0 && currentHalfSecondBlock < m_halfSecondStartIndices.size()) {
        const int currentHalfSecondStartIndex = m_halfSecondStartIndices.at(currentHalfSecondBlock);
        if (currentHalfSecondStartIndex >= 0 && currentHalfSecondStartIndex <= clampedIndex) {
            const int rawCount = clampedIndex - currentHalfSecondStartIndex + 1;
            result.first += xData.mid(currentHalfSecondStartIndex, rawCount);
            result.second += yData.mid(currentHalfSecondStartIndex, rawCount);
        }
    }
    return result;
}

bool MainWindow::shouldRenderReplayPlot(const QWidget *plotBox, const QCustomPlot *plot) const
{
    if (!plot) {
        return false;
    }
    if (m_suspendReplayPlotRendering) {
        return false;
    }
    if (!isReplayTabActive()) {
        return true;
    }
    if (!m_replayPlotScrollArea || !m_replayPlotScrollArea->viewport() || !plotBox) {
        return false;
    }

    const QWidget *viewport = m_replayPlotScrollArea->viewport();
    const QPoint topLeft = plotBox->mapTo(const_cast<QWidget *>(viewport), QPoint(0, 0));
    const QRect mappedRect(topLeft, plotBox->size());
    return mappedRect.intersects(viewport->rect());
}

bool MainWindow::isStaticOverviewMode(const QComboBox *viewModeComboBox) const
{
    if (!viewModeComboBox) {
        return true;
    }
    return viewModeComboBox->currentData().toInt() == kStaticOverviewMode;
}

QString MainWindow::formatTimestamp(const ReplaySample &sample) const
{
    if (sample.hasTimestamp) {
        return sample.timestampUtc.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss 'UTC'"));
    }

    if (sample.playbackMs >= 0) {
        const int totalSeconds = static_cast<int>(sample.playbackMs / 1000);
        const int hours = totalSeconds / 3600;
        const int minutes = (totalSeconds % 3600) / 60;
        const int seconds = totalSeconds % 60;
        const int millis = static_cast<int>(sample.playbackMs % 1000);
        return QStringLiteral("%1:%2:%3.%4")
                .arg(hours, 2, 10, QLatin1Char('0'))
                .arg(minutes, 2, 10, QLatin1Char('0'))
                .arg(seconds, 2, 10, QLatin1Char('0'))
                .arg(millis, 3, 10, QLatin1Char('0'));
    }

    return QStringLiteral("--");
}

void MainWindow::setControlsEnabled(bool enabled)
{
    m_playButton->setEnabled(enabled);
    m_timelineSlider->setEnabled(enabled);
    m_speedCombo->setEnabled(enabled);
    m_accelViewModeCombo->setEnabled(enabled);
    m_accelDisplayCombo->setEnabled(enabled);
    m_gyroViewModeCombo->setEnabled(enabled);
    m_gyroDisplayCombo->setEnabled(enabled);
    m_poseViewModeCombo->setEnabled(enabled);
    m_poseDisplayCombo->setEnabled(enabled);
    if (m_noiseViewModeCombo) {
        m_noiseViewModeCombo->setEnabled(enabled);
    }
    if (m_speedViewModeCombo) {
        m_speedViewModeCombo->setEnabled(enabled);
    }
    if (m_accelCarouselRadio) {
        m_accelCarouselRadio->setEnabled(enabled);
    }
    if (m_gyroCarouselRadio) {
        m_gyroCarouselRadio->setEnabled(enabled);
    }
    if (m_poseCarouselRadio) {
        m_poseCarouselRadio->setEnabled(enabled);
    }
    if (m_noiseCarouselRadio) {
        m_noiseCarouselRadio->setEnabled(enabled);
    }
    if (m_speedCarouselRadio) {
        m_speedCarouselRadio->setEnabled(enabled);
    }
    if (m_accelerationRangeSlider) {
        m_accelerationRangeSlider->setEnabled(enabled);
    }
    if (m_poseRollExaggerationSlider) {
        m_poseRollExaggerationSlider->setEnabled(enabled);
    }
    if (m_posePitchExaggerationSlider) {
        m_posePitchExaggerationSlider->setEnabled(enabled);
    }
    if (m_poseYawExaggerationSlider) {
        m_poseYawExaggerationSlider->setEnabled(enabled);
    }
    m_jumpHourSpin->setEnabled(enabled);
    m_jumpMinuteSpin->setEnabled(enabled);
    m_jumpSecondSpin->setEnabled(enabled);
    m_jumpMillisecondSpin->setEnabled(enabled);
    m_jumpButton->setEnabled(enabled);
    if (m_jumpBackwardButton) {
        m_jumpBackwardButton->setEnabled(enabled);
    }
    if (m_jumpForwardButton) {
        m_jumpForwardButton->setEnabled(enabled);
    }
    updatePlotCarouselTimerState();
}

void MainWindow::jumpByOffsetMs(qint64 deltaMs)
{
    if (m_samples.isEmpty()) {
        return;
    }

    qint64 currentPlaybackMs = 0;
    if (m_currentIndex >= 0 && m_currentIndex < m_samples.size()) {
        currentPlaybackMs = m_samples.at(m_currentIndex).playbackMs;
    }

    const qint64 maxPlaybackMs = m_samples.isEmpty() ? 0 : m_samples.constLast().playbackMs;
    const qint64 targetPlaybackMs = qBound<qint64>(0, currentPlaybackMs + deltaMs, maxPlaybackMs);
    const int targetIndex = findNearestSampleIndex(targetPlaybackMs);
    if (targetIndex >= 0) {
        updateUiForIndex(targetIndex);
    }
}

void MainWindow::updatePlaybackButtonText()
{
    if (!m_playButton) {
        return;
    }

    const bool isPlaying = m_playbackTimer && m_playbackTimer->isActive();
    m_playButton->setText(isPlaying ? QStringLiteral("暂停") : QStringLiteral("播放"));
}

void MainWindow::updateFrameCostLabel()
{
    if (!m_frameCostLabel) {
        return;
    }
    m_frameCostLabel->setStyleSheet(QString());
    const QString frameCostText = QStringLiteral("单帧耗时: -- ms");
    m_frameCostLabel->setText(frameCostText);
    expandLabelMinimumWidth(m_frameCostLabel, &m_frameCostMinWidth, frameCostText);
}

void MainWindow::scheduleFrameCostMeasurement()
{
    if (!m_frameCostLabel) {
        return;
    }

    ++m_frameCostMeasurementId;
    const qint64 measurementId = m_frameCostMeasurementId;
    m_frameCostTimer.restart();
    QTimer::singleShot(0, this, [this, measurementId]() {
        if (!m_frameCostLabel || measurementId != m_frameCostMeasurementId || !m_frameCostTimer.isValid()) {
            return;
        }

        const double elapsedMs = static_cast<double>(m_frameCostTimer.nsecsElapsed()) / 1000000.0;
        m_frameCostLabel->setStyleSheet(elapsedMs > m_frameCostWarningThresholdMs
                                        ? QStringLiteral("color: red;")
                                        : QString());
        const QString frameCostText = QStringLiteral("单帧耗时: %1 ms").arg(formatNumber(elapsedMs, 1));
        m_frameCostLabel->setText(frameCostText);
        expandLabelMinimumWidth(m_frameCostLabel, &m_frameCostMinWidth, frameCostText);
    });
}

QVector<int> MainWindow::selectedSeriesIndices(const QComboBox *comboBox, int seriesCount) const
{
    QVector<int> result;
    if (seriesCount <= 0) {
        return result;
    }

    if (!comboBox || comboBox->currentIndex() <= 0) {
        result.reserve(seriesCount);
        for (int index = 0; index < seriesCount; ++index) {
            result.push_back(index);
        }
        return result;
    }

    const int selectedIndex = comboBox->currentIndex() - 1;
    if (selectedIndex >= 0 && selectedIndex < seriesCount) {
        result.push_back(selectedIndex);
    }
    return result;
}

void MainWindow::syncJumpTimeControls(qint64 playbackMs)
{
    if (!m_jumpHourSpin || !m_jumpMinuteSpin || !m_jumpSecondSpin || !m_jumpMillisecondSpin) {
        return;
    }

    const qint64 safePlaybackMs = qMax<qint64>(0, playbackMs);
    const qint64 jumpResolutionMs = qMax<qint64>(1, m_sampleIntervalMs);
    const qint64 alignedPlaybackMs = ((safePlaybackMs + jumpResolutionMs / 2) / jumpResolutionMs) * jumpResolutionMs;
    const int hours = static_cast<int>(alignedPlaybackMs / 3600000LL);
    const int minutes = static_cast<int>((alignedPlaybackMs % 3600000LL) / 60000LL);
    const int seconds = static_cast<int>((alignedPlaybackMs % 60000LL) / 1000LL);
    const int milliseconds = static_cast<int>(alignedPlaybackMs % 1000LL);

    const QSignalBlocker hourBlocker(m_jumpHourSpin);
    const QSignalBlocker minuteBlocker(m_jumpMinuteSpin);
    const QSignalBlocker secondBlocker(m_jumpSecondSpin);
    const QSignalBlocker millisecondBlocker(m_jumpMillisecondSpin);

    m_jumpHourSpin->setValue(qMin(hours, m_jumpHourSpin->maximum()));
    m_jumpMinuteSpin->setValue(minutes);
    m_jumpSecondSpin->setValue(seconds);
    m_jumpMillisecondSpin->setValue(qMin(milliseconds, m_jumpMillisecondSpin->maximum()));
}

void MainWindow::updateJumpTimeRange()
{
    if (!m_jumpHourSpin || !m_jumpMinuteSpin || !m_jumpSecondSpin || !m_jumpMillisecondSpin) {
        return;
    }

    const qint64 maxPlaybackMs = m_samples.isEmpty() ? 0 : qMax<qint64>(0, m_samples.last().playbackMs);
    const qint64 jumpResolutionMs = qMax<qint64>(1, m_sampleIntervalMs);
    const qint64 alignedMaxPlaybackMs = ((maxPlaybackMs + jumpResolutionMs - 1) / jumpResolutionMs) * jumpResolutionMs;
    m_jumpHourSpin->setRange(0, static_cast<int>(alignedMaxPlaybackMs / 3600000LL));
    m_jumpMinuteSpin->setRange(0, 59);
    m_jumpSecondSpin->setRange(0, 59);
    m_jumpMillisecondSpin->setRange(0, 999);
    m_jumpMillisecondSpin->setSingleStep(static_cast<int>(jumpResolutionMs));
}

int MainWindow::findNearestSampleIndex(qint64 playbackMs) const
{
    if (m_samples.isEmpty()) {
        return -1;
    }

    const auto begin = m_samples.cbegin();
    const auto end = m_samples.cend();
    const auto upperIt = std::lower_bound(
                begin,
                end,
                playbackMs,
                [](const ReplaySample &sample, qint64 value) {
        return sample.playbackMs < value;
    });

    if (upperIt == begin) {
        return 0;
    }

    if (upperIt == end) {
        return m_samples.size() - 1;
    }

    const int upperIndex = static_cast<int>(upperIt - begin);
    const int lowerIndex = upperIndex - 1;
    const qint64 lowerDiff = qAbs(m_samples.at(lowerIndex).playbackMs - playbackMs);
    const qint64 upperDiff = qAbs(m_samples.at(upperIndex).playbackMs - playbackMs);
    return upperDiff < lowerDiff ? upperIndex : lowerIndex;
}
