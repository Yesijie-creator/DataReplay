#ifndef DATAFILEPARSER_H
#define DATAFILEPARSER_H

#include <QDateTime>
#include <QMetaType>
#include <QString>
#include <QVector>

struct CorrectedImuSample
{
    bool valid = false;
    double accelZG = 0.0;
    double biasZG = 0.0;
    bool stationaryBiasApplied = false;
    bool closureBiasApplied = false;
};

struct ReplaySample
{
    int sequence = 0;
    qint64 playbackMs = 0;
    bool hasTimestamp = false;
    QDateTime timestampUtc;

    bool hasGps = false;
    quint8 gpsFixMode = 0;
    quint8 gpsSvNum = 0;
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double altitudeM = 0.0;
    double vxMps = 0.0;
    double vyMps = 0.0;
    double vzMps = 0.0;
    double noiseDbA = 0.0;

    bool hasImu = false;
    double gyroXDeg = 0.0;
    double gyroYDeg = 0.0;
    double gyroZDeg = 0.0;
    double accelXG = 0.0;
    double accelYG = 0.0;
    double accelZG = 0.0;
    double pressureHpa = 0.0;
    double temperatureC = 0.0;
    quint8 statusFlag = 0;

    bool poseValid = false;
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double yawDeg = 0.0;
    CorrectedImuSample correctedImu;
    double derivedSpeedZGs = 0.0;
    double derivedMileageZGss = 0.0;
    double derivedMileageM = 0.0;

    double relativeEastM = 0.0;
    double relativeNorthM = 0.0;
    double relativeAltitudeM = 0.0;
    double horizontalDistanceM = 0.0;
};

struct MileageCorrectionStation
{
    QString stationName;
    double upCenterMileageM = 0.0;
    double downCenterMileageM = 0.0;
    double upIntervalM = 0.0;
    double downIntervalM = 0.0;
    double accumulatedUpMileageM = 0.0;
};

struct MileageCorrectionTable
{
    QString sourceFilePath;
    QString sheetName;
    QVector<MileageCorrectionStation> stations;

    bool isValid() const
    {
        return !sheetName.isEmpty() && !stations.isEmpty();
    }
};

struct PoseSweepSection
{
    double centerX = 0.0;
    double centerY = 0.0;
    double centerZ = 0.0;
    double rightX = 1.0;
    double rightY = 0.0;
    double rightZ = 0.0;
    double upX = 0.0;
    double upY = 1.0;
    double upZ = 0.0;
    double halfWidth = 0.0;
    double halfHeight = 0.0;
    qint64 playbackMs = 0;
};

struct PoseSceneGeometry
{
    QVector<PoseSweepSection> leftRailSections;
    QVector<PoseSweepSection> rightRailSections;
};

struct ParseResult
{
    QVector<ReplaySample> samples;
    PoseSceneGeometry poseSceneGeometry;
    qint64 resolvedSampleIntervalMs = 5;
    double resolvedSampleIntervalSeconds = 0.005;
    int parsedC1 = 0;
    int parsedC2 = 0;
    int invalidFrames = 0;
    int unknownBytesSkipped = 0;
    bool hasFirstC1Timestamp = false;
    bool hasLastC1Timestamp = false;
    qint64 firstC1TimestampMs = -1;
    qint64 lastC1TimestampMs = -1;
    bool sampleIntervalFallbackUsed = false;
    bool mileageCorrectionApplied = false;
    int mileageCorrectionStopCount = 0;
    int mileageCorrectionOverflowCount = 0;
    QString errorMessage;
    QString warningMessage;
};

class DataFileParser
{
public:
    ParseResult parseFile(const QString &filePath,
                          const MileageCorrectionTable *mileageCorrection = nullptr) const;
    bool loadMileageCorrectionFile(const QString &filePath,
                                   MileageCorrectionTable *table,
                                   QString *errorMessage) const;

private:
    static constexpr int kFrameSize = 38;

    static bool isHeader(const QByteArray &bytes, int offset);
    static quint8 checksum(const QByteArray &bytes, int offset);
    static qint32 readInt32LE(const QByteArray &bytes, int offset);
    static quint16 readUInt16LE(const QByteArray &bytes, int offset);
    static float readFloatLE(const QByteArray &bytes, int offset);
    static QDateTime readTimestampUtc(const QByteArray &bytes, int offset, bool *ok);
};

Q_DECLARE_METATYPE(ReplaySample)

#endif // DATAFILEPARSER_H
