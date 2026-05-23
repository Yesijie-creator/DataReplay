#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "datafileparser.h"

#include <QElapsedTimer>
#include <QMainWindow>
#include <QStringList>

class QLabel;
class QPushButton;
class QRadioButton;
class QSlider;
class QTimer;
class QTableWidget;
class QComboBox;
class QSpinBox;
class QTabWidget;
class QScrollArea;
class QGroupBox;

class OpenGLWidgetTest;
class AccelerationVisualizationWidget;
class PoseSimulationWidget;
class QCustomPlot;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private slots:
    void openDataFile();
    void openMileageCorrectionFile();
    void togglePlayback();
    void play();
    void pause();
    void playbackStep();
    void sliderChanged(int value);
    void speedChanged(int index);
    void plotDisplayModeChanged();
    void plotCarouselToggled(bool checked);
    void plotModeSelectionChanged();
    void tabChanged(int index);
    void jumpToTime();
    void jumpBackward15Seconds();
    void jumpForward30Seconds();
    void accelerationRangeChanged(int value);
    void poseRollExaggerationChanged(int value);
    void posePitchExaggerationChanged(int value);
    void poseYawExaggerationChanged(int value);

private:
    struct PlotCarouselContext
    {
        QComboBox *viewModeCombo = nullptr;
        QComboBox *displayCombo = nullptr;
        QRadioButton *carouselRadio = nullptr;
    };

    struct StaticOverviewSeriesCache
    {
        int targetWidth = -1;
        QVector<QPair<QVector<double>, QVector<double> > > downsampledSeries;
    };

    void buildUi();
    void setSamples(const ParseResult &result, const QString &filePath);
    void updateUiForIndex(int index);
    void resetUiState();
    void updateSummary(const ParseResult &result, const QString &filePath);
    void refreshPlots();
    QString formatTimestamp(const ReplaySample &sample) const;
    void setControlsEnabled(bool enabled);
    QVector<int> selectedSeriesIndices(const QComboBox *comboBox, int seriesCount) const;
    void syncJumpTimeControls(qint64 playbackMs);
    void updateJumpTimeRange();
    int findNearestSampleIndex(qint64 playbackMs) const;
    void updatePlaybackButtonText();
    void updateFrameCostLabel();
    void scheduleFrameCostMeasurement();
    void advancePlotCarousels();
    void advanceInfoCarousel();
    void jumpByOffsetMs(qint64 deltaMs);
    void refreshNoisePlot();
    void refreshSpeedPlot();
    void configurePlot(QCustomPlot *plot);
    void rebuildPlotDataCache();
    void clearPlotDataCache();
    void updateTopInfoDisplay();
    void updateStatusInfoDisplay();
    void updateInfoCarouselTimerState();
    void resetInfoCarouselState();
    QStringList buildTopInfoItems(const QString &filePath, const QString &summaryItem, const QString &warningMessage) const;
    QStringList buildStatusInfoItems(const QString &summaryItem, const QString &warningMessage) const;
    void expandLabelMinimumWidth(QLabel *label, int *cachedWidth, const QString &text);
    void initializePlotCarousels();
    void updatePlotCarouselTimerState();
    void advancePlotCarousel(PlotCarouselContext *context);
    void setPlotCarouselCombination(PlotCarouselContext *context, int combinationIndex);
    void setPlotCarouselEnabled(PlotCarouselContext *context, bool enabled);
    int currentPlotCarouselCombinationIndex(const PlotCarouselContext *context) const;
    int plotCarouselCombinationCount(const PlotCarouselContext *context) const;
    PlotCarouselContext *plotCarouselContextForObject(const QObject *object);
    void refreshSinglePlot(QCustomPlot *plot,
                           const QComboBox *seriesComboBox,
                           const QComboBox *viewModeComboBox,
                           const QVector<QVector<double> > &series,
                           const QVector<QVector<QVector<int> > > &sampledHistoryIndexCaches,
                           const QVector<QString> &names,
                           const QString &yLabel);
    void buildStaticOverviewPlot(QCustomPlot *plot,
                                 const QVector<double> &xData,
                                 const QVector<QVector<double> > &series,
                                 const QVector<QString> &names,
                                 const QVector<int> &visibleIndices,
                                 const QString &yLabel);
    void buildDynamicFollowPlot(QCustomPlot *plot,
                                const QVector<double> &xData,
                                const QVector<QVector<double> > &series,
                                const QVector<QVector<QVector<int> > > &sampledHistoryIndexCaches,
                                const QVector<QString> &names,
                                const QVector<int> &visibleIndices,
                                const QString &yLabel,
                                int currentIndex);
    int staticOverviewTargetWidth(const QCustomPlot *plot) const;
    int downsamplePointBudgetForWidth(int targetWidth) const;
    QPair<QVector<double>, QVector<double> > buildDownsampledStaticOverviewSeries(const QVector<double> &xData,
                                                                                  const QVector<double> &yData,
                                                                                  int targetWidth) const;
    StaticOverviewSeriesCache *staticOverviewCacheForPlot(const QCustomPlot *plot);
    QPair<QVector<double>, QVector<double> > buildDynamicWindowSeries(const QVector<double> &xData,
                                                                      const QVector<double> &yData,
                                                                      const QVector<QVector<int> > &sampledHistoryIndexCache,
                                                                      int currentIndex) const;
    bool shouldRenderReplayPlot(const QWidget *plotBox, const QCustomPlot *plot) const;
    bool isStaticOverviewMode(const QComboBox *viewModeComboBox) const;
    bool isReplayTabActive() const;
    bool isAccelerationTabActive() const;
    bool isPoseTabActive() const;
    void refreshReplayPageForCurrentSample();
    void refreshCurrentVisiblePage();

    QVector<ReplaySample> m_samples;
    PoseSceneGeometry m_poseSceneGeometry;
    DataFileParser m_parser;
    QTimer *m_playbackTimer = nullptr;
    int m_currentIndex = -1;
    double m_playbackSpeed = 5.0;
    qint64 m_sampleIntervalMs = 5;
    double m_sampleIntervalSeconds = 0.005;
    double m_frameCostWarningThresholdMs = 5.0;
    QElapsedTimer m_frameCostTimer;
    qint64 m_frameCostMeasurementId = 0;

    QLabel *m_topInfoLabel = nullptr;
    QLabel *m_statusCarouselLabel = nullptr;
    QLabel *m_currentTimeLabel = nullptr;
    QLabel *m_poseLabel = nullptr;
    QLabel *m_frameCostLabel = nullptr;
    QLabel *m_accelerationRangeLabel = nullptr;
    QLabel *m_poseRollExaggerationLabel = nullptr;
    QLabel *m_posePitchExaggerationLabel = nullptr;
    QLabel *m_poseYawExaggerationLabel = nullptr;

    QSlider *m_timelineSlider = nullptr;
    QSlider *m_accelerationRangeSlider = nullptr;
    QSlider *m_poseRollExaggerationSlider = nullptr;
    QSlider *m_posePitchExaggerationSlider = nullptr;
    QSlider *m_poseYawExaggerationSlider = nullptr;

    QPushButton *m_openButton = nullptr;
    QPushButton *m_openMileageCorrectionButton = nullptr;
    QPushButton *m_playButton = nullptr;
    QPushButton *m_jumpButton = nullptr;
    QPushButton *m_jumpBackwardButton = nullptr;
    QPushButton *m_jumpForwardButton = nullptr;

    QComboBox *m_speedCombo = nullptr;
    QComboBox *m_accelViewModeCombo = nullptr;
    QComboBox *m_accelDisplayCombo = nullptr;
    QComboBox *m_gyroViewModeCombo = nullptr;
    QComboBox *m_gyroDisplayCombo = nullptr;
    QComboBox *m_poseViewModeCombo = nullptr;
    QComboBox *m_poseDisplayCombo = nullptr;
    QComboBox *m_noiseViewModeCombo = nullptr;
    QComboBox *m_speedViewModeCombo = nullptr;
    QRadioButton *m_accelCarouselRadio = nullptr;
    QRadioButton *m_gyroCarouselRadio = nullptr;
    QRadioButton *m_poseCarouselRadio = nullptr;
    QRadioButton *m_noiseCarouselRadio = nullptr;
    QRadioButton *m_speedCarouselRadio = nullptr;

    QSpinBox *m_jumpHourSpin = nullptr;
    QSpinBox *m_jumpMinuteSpin = nullptr;
    QSpinBox *m_jumpSecondSpin = nullptr;
    QSpinBox *m_jumpMillisecondSpin = nullptr;

    QCustomPlot *m_accelPlot = nullptr;
    QCustomPlot *m_gyroPlot = nullptr;
    QCustomPlot *m_posePlot = nullptr;
    QCustomPlot *m_noisePlot = nullptr;
    QCustomPlot *m_speedPlot = nullptr;

    QVector<double> m_plotTimeData;
    QVector<double> m_accelXData;
    QVector<double> m_accelYData;
    QVector<double> m_accelZData;
    QVector<double> m_gyroXData;
    QVector<double> m_gyroYData;
    QVector<double> m_gyroZData;
    QVector<double> m_rollData;
    QVector<double> m_pitchData;
    QVector<double> m_yawData;
    QVector<double> m_noiseData;
    QVector<double> m_speedZData;
    QVector<int> m_halfSecondStartIndices;
    QVector<int> m_halfSecondEndIndices;
    QVector<QVector<int> > m_accelXHistorySampledIndices;
    QVector<QVector<int> > m_accelYHistorySampledIndices;
    QVector<QVector<int> > m_accelZHistorySampledIndices;
    QVector<QVector<int> > m_gyroXHistorySampledIndices;
    QVector<QVector<int> > m_gyroYHistorySampledIndices;
    QVector<QVector<int> > m_gyroZHistorySampledIndices;
    QVector<QVector<int> > m_rollHistorySampledIndices;
    QVector<QVector<int> > m_pitchHistorySampledIndices;
    QVector<QVector<int> > m_yawHistorySampledIndices;
    QVector<QVector<int> > m_noiseHistorySampledIndices;
    QVector<QVector<int> > m_speedZHistorySampledIndices;
    PlotCarouselContext m_accelCarouselContext;
    PlotCarouselContext m_gyroCarouselContext;
    PlotCarouselContext m_poseCarouselContext;
    PlotCarouselContext m_noiseCarouselContext;
    PlotCarouselContext m_speedCarouselContext;
    StaticOverviewSeriesCache m_accelOverviewCache;
    StaticOverviewSeriesCache m_gyroOverviewCache;
    StaticOverviewSeriesCache m_poseOverviewCache;
    StaticOverviewSeriesCache m_noiseOverviewCache;
    StaticOverviewSeriesCache m_speedOverviewCache;

    QTableWidget *m_valueTable = nullptr;
    QTableWidget *m_solutionTable = nullptr;
    QTabWidget *m_mainTabs = nullptr;
    QTimer *m_infoCarouselTimer = nullptr;
    QTimer *m_plotCarouselTimer = nullptr;
    QWidget *m_replayPage = nullptr;
    QScrollArea *m_replayPlotScrollArea = nullptr;
    QWidget *m_replayPlotContainer = nullptr;
    QGroupBox *m_noisePlotBox = nullptr;
    QGroupBox *m_speedPlotBox = nullptr;
    AccelerationVisualizationWidget *m_accelerationVisualizationView = nullptr;
    PoseSimulationWidget *m_poseSimulationView = nullptr;
    OpenGLWidgetTest *m_openGlWidgetTestView = nullptr;
    QStringList m_topInfoItems;
    QStringList m_statusInfoItems;
    QString m_currentSensorFilePath;
    QString m_currentMileageCorrectionFilePath;
    MileageCorrectionTable m_mileageCorrectionTable;
    bool m_hasMileageCorrectionTable = false;
    int m_topInfoIndex = 0;
    int m_statusInfoIndex = 0;
    int m_topInfoMinWidth = 0;
    int m_statusInfoMinWidth = 0;
    int m_currentTimeMinWidth = 0;
    int m_poseMinWidth = 0;
    int m_frameCostMinWidth = 0;
    bool m_topInfoHovered = false;
    bool m_statusInfoHovered = false;
    bool m_plotCarouselProgrammaticChange = false;
    bool m_suspendReplayPlotRendering = false;
    qint64 m_replayPlotScrollSuspendToken = 0;
};

#endif // MAINWINDOW_H
