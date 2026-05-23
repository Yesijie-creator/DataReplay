#include "datafileparser.h"

#include "xlsxdocument.h"

#include <QFile>
#include <QMatrix4x4>
#include <QStringList>
#include <QVector3D>
#include <QVector4D>
#include <QVariant>

#include <QtMath>

#include <cstring>
#include <limits>

namespace
{
static constexpr qint64 kDefaultSampleIntervalMs = 5;
static constexpr double kDefaultSampleIntervalSeconds = static_cast<double>(kDefaultSampleIntervalMs) / 1000.0;
static constexpr double kPoseCorrectionAccelNormToleranceG = 0.08;
static constexpr double kPoseCorrectionGyroThresholdDegPerSec = 3.0;
static constexpr double kGyroBiasUpdateAlpha = 0.995;
static constexpr qint64 kBiasStationarySearchMs = 4000;
static constexpr qint64 kBiasStationaryWindowMs = 800;
static constexpr double kBiasStationaryGyroThresholdDegPerSec = 1.5;
static constexpr double kBiasStationaryAccelStdDevThresholdG = 0.015;
static constexpr double kBiasStationaryAccelMeanThresholdG = 0.2;
static constexpr double kBiasClosureResidualThresholdGs = 0.02;
static constexpr double kSpeedZeroingAccelThresholdG = 0.008;
static constexpr qint64 kSpeedZeroingStationaryDurationMs = 15000;
static constexpr qint64 kSpeedZeroingCooldownMs = 60000;
static constexpr float kGroundGridCellSize = 5.0f;
static constexpr float kTrainLength = 2.0f;
static constexpr float kTrainWidth = kTrainLength * (1.55f / 3.05f);
static constexpr float kTrainHeight = kTrainLength * (0.60f / 3.05f);
static constexpr float kGroundNearThreshold = 0.05f * kGroundGridCellSize;
static constexpr float kTrainBaseCenterY = kGroundNearThreshold + kTrainHeight * 0.5f + 0.10f;
static constexpr float kRailCrossSectionScale = 4.0f;
static constexpr float kRailWidth = 0.02f * kRailCrossSectionScale;
static constexpr float kRailThickness = 0.02f * kRailCrossSectionScale;
static constexpr float kRailCenterOffsetX = kTrainWidth * 0.5f - kRailWidth * 0.5f;
static const QStringList kMileageCorrectionHeaders = {
    QStringLiteral("车站"),
    QStringLiteral("上行车站中心里程"),
    QStringLiteral("下行车站中心里程"),
    QStringLiteral("上行站间距"),
    QStringLiteral("下行站间距")
};

struct HeaderMatch
{
    bool valid = false;
    int row = 0;
    int firstColumn = 0;
};

struct MotionRebuildSummary
{
    bool stationaryZeroingApplied = false;
    int stationaryZeroingRejectedCount = 0;
};

bool isReasonableTimestamp(const QDateTime &timestampUtc)
{
    return timestampUtc.isValid();
}

double normalizeAngleDeg(double value)
{
    while (value > 180.0) {
        value -= 360.0;
    }
    while (value < -180.0) {
        value += 360.0;
    }
    return value;
}

double degreesToRadians(double value)
{
    return qDegreesToRadians(value);
}

QMatrix4x4 buildPoseMatrix(float rollDeg, float pitchDeg, float yawDeg)
{
    QMatrix4x4 pose;
    pose.rotate(yawDeg, 0.0f, 1.0f, 0.0f);
    pose.rotate(pitchDeg, 1.0f, 0.0f, 0.0f);
    pose.rotate(rollDeg, 0.0f, 0.0f, 1.0f);
    return pose;
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

QVector3D normalizedAxis(const QMatrix4x4 &pose, const QVector3D &localAxis)
{
    QVector3D axis = (pose * QVector4D(localAxis, 0.0f)).toVector3D();
    if (axis.lengthSquared() <= 0.0f) {
        return localAxis;
    }
    axis.normalize();
    return axis;
}

PoseSweepSection buildSweepSection(const QVector3D &trainPosition,
                                   const QMatrix4x4 &pose,
                                   const QVector3D &localCenter,
                                   float halfWidth,
                                   float halfHeight,
                                   qint64 playbackMs)
{
    PoseSweepSection section;
    const QVector3D worldCenter = trainPosition + (pose * QVector4D(localCenter, 0.0f)).toVector3D();
    const QVector3D right = normalizedAxis(pose, QVector3D(1.0f, 0.0f, 0.0f));
    const QVector3D up = normalizedAxis(pose, QVector3D(0.0f, 1.0f, 0.0f));

    section.centerX = worldCenter.x();
    section.centerY = worldCenter.y();
    section.centerZ = worldCenter.z();
    section.rightX = right.x();
    section.rightY = right.y();
    section.rightZ = right.z();
    section.upX = up.x();
    section.upY = up.y();
    section.upZ = up.z();
    section.halfWidth = halfWidth;
    section.halfHeight = halfHeight;
    section.playbackMs = playbackMs;
    return section;
}

double rollFromAccelDeg(double accelXG, double accelYG)
{
    return qRadiansToDegrees(qAtan2(accelYG, accelXG));
}

double pitchFromAccelDeg(double accelXG, double accelYG, double accelZG)
{
    return qRadiansToDegrees(qAtan2(-accelZG,
                                    qSqrt(accelXG * accelXG + accelYG * accelYG)));
}

bool shouldApplyPoseCorrection(double accelNorm,
                               double correctedGyroYDegPerSec,
                               double correctedGyroZDegPerSec)
{
    return qAbs(accelNorm - 1.0) <= kPoseCorrectionAccelNormToleranceG
            && qAbs(correctedGyroYDegPerSec) <= kPoseCorrectionGyroThresholdDegPerSec
            && qAbs(correctedGyroZDegPerSec) <= kPoseCorrectionGyroThresholdDegPerSec;
}

void appendWarningMessage(QString *target, const QString &message)
{
    if (!target || message.isEmpty()) {
        return;
    }
    if (!target->isEmpty()) {
        *target += QStringLiteral("；");
    }
    *target += message;
}

QVariant readCellValue(QXlsx::Document *document, int row, int column)
{
    if (!document) {
        return QVariant();
    }

    const auto cell = document->cellAt(row, column);
    if (cell) {
        return cell->readValue();
    }
    return document->read(row, column);
}

QString normalizedCellText(const QVariant &value)
{
    return value.toString().trimmed();
}

bool parsePlainNumber(const QString &text, double *value)
{
    if (!value) {
        return false;
    }

    QString normalized = text.trimmed();
    normalized.remove(QLatin1Char(','));
    if (normalized.isEmpty()) {
        return false;
    }

    bool ok = false;
    const double parsed = normalized.toDouble(&ok);
    if (!ok) {
        return false;
    }
    *value = parsed;
    return true;
}

bool parseMileageValue(const QVariant &value, double *mileageM)
{
    if (!mileageM) {
        return false;
    }

    if (!value.isValid() || value.isNull()) {
        return false;
    }

    const QString text = normalizedCellText(value);
    if (text.isEmpty()) {
        return false;
    }

    QString normalized = text;
    normalized.remove(QLatin1Char(','));
    if (normalized.contains(QLatin1Char('+'))) {
        const QStringList parts = normalized.split(QLatin1Char('+'), Qt::KeepEmptyParts);
        if (parts.size() != 2) {
            return false;
        }

        bool kmOk = false;
        bool meterOk = false;
        const double km = parts.at(0).trimmed().toDouble(&kmOk);
        const double meter = parts.at(1).trimmed().toDouble(&meterOk);
        if (!kmOk || !meterOk) {
            return false;
        }
        *mileageM = km * 1000.0 + meter;
        return true;
    }

    return parsePlainNumber(normalized, mileageM);
}

bool parseIntervalValue(const QVariant &value, double *intervalM)
{
    if (!intervalM) {
        return false;
    }

    if (!value.isValid() || value.isNull()) {
        return false;
    }

    return parsePlainNumber(normalizedCellText(value), intervalM);
}

HeaderMatch findMileageCorrectionHeader(QXlsx::Document *document)
{
    HeaderMatch result;
    if (!document) {
        return result;
    }

    const QXlsx::CellRange range = document->dimension();
    if (range.firstRow() == 0 && range.lastRow() == 0
            && range.firstColumn() == 0 && range.lastColumn() == 0) {
        return result;
    }

    for (int row = range.firstRow(); row <= range.lastRow(); ++row) {
        for (int column = range.firstColumn();
             column + kMileageCorrectionHeaders.size() - 1 <= range.lastColumn();
             ++column) {
            bool matched = true;
            for (int offset = 0; offset < kMileageCorrectionHeaders.size(); ++offset) {
                const QString headerText = normalizedCellText(readCellValue(document, row, column + offset));
                if (headerText != kMileageCorrectionHeaders.at(offset)) {
                    matched = false;
                    break;
                }
            }

            if (matched) {
                result.valid = true;
                result.row = row;
                result.firstColumn = column;
                return result;
            }
        }
    }

    return result;
}

bool loadMileageCorrectionSheet(QXlsx::Document *document,
                                const QString &sheetName,
                                MileageCorrectionTable *table,
                                QString *errorMessage)
{
    if (!document || !table) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("里程校正文件读取参数无效。");
        }
        return false;
    }

    const HeaderMatch header = findMileageCorrectionHeader(document);
    if (!header.valid) {
        return false;
    }

    const QXlsx::CellRange range = document->dimension();
    double accumulatedUpMileageM = 0.0;
    QVector<MileageCorrectionStation> stations;
    for (int row = header.row + 1; row <= range.lastRow(); ++row) {
        const QString stationName = normalizedCellText(readCellValue(document, row, header.firstColumn));
        const QVariant upCenterValue = readCellValue(document, row, header.firstColumn + 1);
        const QVariant downCenterValue = readCellValue(document, row, header.firstColumn + 2);
        const QVariant upIntervalValue = readCellValue(document, row, header.firstColumn + 3);
        const QVariant downIntervalValue = readCellValue(document, row, header.firstColumn + 4);
        const bool allEmpty = stationName.isEmpty()
                && normalizedCellText(upCenterValue).isEmpty()
                && normalizedCellText(downCenterValue).isEmpty()
                && normalizedCellText(upIntervalValue).isEmpty()
                && normalizedCellText(downIntervalValue).isEmpty();
        if (allEmpty) {
            if (!stations.isEmpty()) {
                break;
            }
            continue;
        }

        MileageCorrectionStation station;
        station.stationName = stationName;
        if (station.stationName.isEmpty()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("工作表 %1 第 %2 行车站名称为空，无法继续读取里程校正表。")
                        .arg(sheetName)
                        .arg(row);
            }
            return false;
        }
        if (!parseMileageValue(upCenterValue, &station.upCenterMileageM)
                || !parseMileageValue(downCenterValue, &station.downCenterMileageM)
                || !parseIntervalValue(upIntervalValue, &station.upIntervalM)
                || !parseIntervalValue(downIntervalValue, &station.downIntervalM)) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("工作表 %1 第 %2 行里程校正数据格式无效。")
                        .arg(sheetName)
                        .arg(row);
            }
            return false;
        }

        accumulatedUpMileageM += station.upIntervalM;
        station.accumulatedUpMileageM = accumulatedUpMileageM;
        stations.push_back(station);
    }

    if (stations.isEmpty()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("工作表 %1 已匹配到表头，但未读取到有效站点数据。")
                    .arg(sheetName);
        }
        return false;
    }

    table->sheetName = sheetName;
    table->stations = stations;
    return true;
}

struct StationaryBiasEstimate
{
    bool valid = false;
    double meanAccelZG = 0.0;
};

StationaryBiasEstimate estimateBoundaryStationaryBias(const QVector<ReplaySample> &samples,
                                                     qint64 sampleIntervalMs,
                                                     bool fromStart)
{
    const qint64 boundedSampleIntervalMs = qMax<qint64>(1, sampleIntervalMs);
    const int windowSamples = qMax(20, static_cast<int>(qCeil(static_cast<double>(kBiasStationaryWindowMs)
                                                              / static_cast<double>(boundedSampleIntervalMs))));
    const int searchSamples = qMax(windowSamples,
                                   static_cast<int>(qCeil(static_cast<double>(kBiasStationarySearchMs)
                                                          / static_cast<double>(boundedSampleIntervalMs))));
    if (samples.size() < windowSamples) {
        return StationaryBiasEstimate();
    }

    const int firstStart = fromStart ? 0 : qMax(0, samples.size() - searchSamples);
    const int lastStart = fromStart ? qMin(samples.size() - windowSamples,
                                           searchSamples - windowSamples)
                                    : samples.size() - windowSamples;
    const int step = fromStart ? 1 : -1;

    for (int start = fromStart ? firstStart : lastStart;
         fromStart ? (start <= lastStart) : (start >= firstStart);
         start += step) {
        double sumAccelZ = 0.0;
        double sumAccelZSquared = 0.0;
        double maxAbsGyroDegPerSec = 0.0;
        bool allImuSamplesValid = true;

        for (int index = start; index < start + windowSamples; ++index) {
            const ReplaySample &sample = samples.at(index);
            if (!sample.hasImu) {
                allImuSamplesValid = false;
                break;
            }

            sumAccelZ += sample.accelZG;
            sumAccelZSquared += sample.accelZG * sample.accelZG;
            maxAbsGyroDegPerSec = qMax(maxAbsGyroDegPerSec, qAbs(sample.gyroXDeg));
            maxAbsGyroDegPerSec = qMax(maxAbsGyroDegPerSec, qAbs(sample.gyroYDeg));
            maxAbsGyroDegPerSec = qMax(maxAbsGyroDegPerSec, qAbs(sample.gyroZDeg));
        }

        if (!allImuSamplesValid) {
            continue;
        }

        const double sampleCount = static_cast<double>(windowSamples);
        const double meanAccelZ = sumAccelZ / sampleCount;
        const double varianceAccelZ = qMax(0.0, sumAccelZSquared / sampleCount - meanAccelZ * meanAccelZ);
        const double stdDevAccelZ = qSqrt(varianceAccelZ);
        if (maxAbsGyroDegPerSec <= kBiasStationaryGyroThresholdDegPerSec
                && stdDevAccelZ <= kBiasStationaryAccelStdDevThresholdG
                && qAbs(meanAccelZ) <= kBiasStationaryAccelMeanThresholdG) {
            StationaryBiasEstimate estimate;
            estimate.valid = true;
            estimate.meanAccelZG = meanAccelZ;
            return estimate;
        }
    }

    return StationaryBiasEstimate();
}

MotionRebuildSummary rebuildDerivedMotion(ParseResult *result,
                                         const MileageCorrectionTable *mileageCorrection)
{
    MotionRebuildSummary summary;
    if (!result || result->samples.isEmpty()) {
        return summary;
    }

    const double sampleIntervalSeconds = result->resolvedSampleIntervalSeconds;
    if (sampleIntervalSeconds <= 0.0) {
        return summary;
    }

    const qint64 boundedSampleIntervalMs = qMax<qint64>(1, result->resolvedSampleIntervalMs);
    const qint64 stationarySamplesRequired = qMax<qint64>(1,
                                                          (kSpeedZeroingStationaryDurationMs
                                                           + boundedSampleIntervalMs - 1)
                                                          / boundedSampleIntervalMs);
    double accumulatedSpeedGs = 0.0;
    double accumulatedMileageZGss = 0.0;
    qint64 consecutiveStationarySamples = 0;
    int stationaryStartSampleIndex = -1;
    qint64 lastAppliedZeroingPlaybackMs = std::numeric_limits<qint64>::min();
    int nextCorrectionStationIndex = 0;

    result->mileageCorrectionApplied = false;
    result->mileageCorrectionStopCount = 0;
    result->mileageCorrectionOverflowCount = 0;

    for (int index = 0; index < result->samples.size(); ++index) {
        ReplaySample &sample = result->samples[index];
        if (sample.hasImu && sample.correctedImu.valid) {
            accumulatedSpeedGs += sample.correctedImu.accelZG * sampleIntervalSeconds;
            accumulatedSpeedGs = qMax(0.0, accumulatedSpeedGs);

            if (qAbs(sample.correctedImu.accelZG) <= kSpeedZeroingAccelThresholdG) {
                if (consecutiveStationarySamples == 0) {
                    stationaryStartSampleIndex = index;
                }
                ++consecutiveStationarySamples;
                if (consecutiveStationarySamples >= stationarySamplesRequired) {
                    const bool cooldownSatisfied = lastAppliedZeroingPlaybackMs == std::numeric_limits<qint64>::min()
                            || (sample.playbackMs - lastAppliedZeroingPlaybackMs) >= kSpeedZeroingCooldownMs;
                    if (cooldownSatisfied) {
                        accumulatedSpeedGs = 0.0;
                        summary.stationaryZeroingApplied = true;
                        lastAppliedZeroingPlaybackMs = sample.playbackMs;

                        const int zeroingStartIndex = qMax(0, stationaryStartSampleIndex);
                        double zeroedMileageZGss = zeroingStartIndex > 0
                                ? result->samples.at(zeroingStartIndex - 1).derivedMileageZGss
                                : 0.0;

                        if (mileageCorrection && mileageCorrection->isValid()) {
                            if (nextCorrectionStationIndex < mileageCorrection->stations.size()) {
                                zeroedMileageZGss = mileageCorrection->stations
                                        .at(nextCorrectionStationIndex)
                                        .accumulatedUpMileageM;
                                result->mileageCorrectionApplied = true;
                                ++result->mileageCorrectionStopCount;
                            } else {
                                ++result->mileageCorrectionOverflowCount;
                            }
                            ++nextCorrectionStationIndex;
                        }

                        accumulatedMileageZGss = zeroedMileageZGss;
                        for (int zeroIndex = zeroingStartIndex; zeroIndex <= index; ++zeroIndex) {
                            result->samples[zeroIndex].derivedSpeedZGs = 0.0;
                            result->samples[zeroIndex].derivedMileageZGss = zeroedMileageZGss;
                        }
                    } else {
                        ++summary.stationaryZeroingRejectedCount;
                    }
                    consecutiveStationarySamples = 0;
                    stationaryStartSampleIndex = -1;
                }
            } else {
                consecutiveStationarySamples = 0;
                stationaryStartSampleIndex = -1;
            }
        } else {
            consecutiveStationarySamples = 0;
            stationaryStartSampleIndex = -1;
        }

        if (sample.hasImu && sample.correctedImu.valid) {
            accumulatedMileageZGss += accumulatedSpeedGs * sampleIntervalSeconds;
        }
        sample.derivedSpeedZGs = accumulatedSpeedGs;
        sample.derivedMileageZGss = accumulatedMileageZGss;
    }

    return summary;
}

void applyCorrectedAccelZ(ParseResult *result,
                         const MileageCorrectionTable *mileageCorrection)
{
    if (!result || result->samples.isEmpty()) {
        return;
    }

    const double sampleIntervalSeconds = result->resolvedSampleIntervalSeconds;
    if (sampleIntervalSeconds <= 0.0) {
        return;
    }

    int imuSampleCount = 0;
    double rawIntegratedSpeedGs = 0.0;
    for (const ReplaySample &sample : qAsConst(result->samples)) {
        if (!sample.hasImu) {
            continue;
        }
        rawIntegratedSpeedGs += sample.accelZG * sampleIntervalSeconds;
        ++imuSampleCount;
    }

    if (imuSampleCount <= 0) {
        return;
    }

    const double totalIntegrationSeconds = static_cast<double>(imuSampleCount) * sampleIntervalSeconds;
    const StationaryBiasEstimate startBias = estimateBoundaryStationaryBias(result->samples,
                                                                            result->resolvedSampleIntervalMs,
                                                                            true);
    const StationaryBiasEstimate endBias = estimateBoundaryStationaryBias(result->samples,
                                                                          result->resolvedSampleIntervalMs,
                                                                          false);

    const bool hasStationaryBias = startBias.valid || endBias.valid;
    const double fallbackConstantBiasG = totalIntegrationSeconds > 0.0
            ? rawIntegratedSpeedGs / totalIntegrationSeconds
            : 0.0;

    for (int index = 0; index < result->samples.size(); ++index) {
        ReplaySample &sample = result->samples[index];
        if (!sample.hasImu) {
            sample.correctedImu = {};
            continue;
        }

        double biasZG = fallbackConstantBiasG;
        if (startBias.valid && endBias.valid) {
            const double alpha = result->samples.size() > 1
                    ? static_cast<double>(index) / static_cast<double>(result->samples.size() - 1)
                    : 0.0;
            biasZG = startBias.meanAccelZG + (endBias.meanAccelZG - startBias.meanAccelZG) * alpha;
        } else if (startBias.valid) {
            biasZG = startBias.meanAccelZG;
        } else if (endBias.valid) {
            biasZG = endBias.meanAccelZG;
        }

        sample.correctedImu.valid = true;
        sample.correctedImu.accelZG = sample.accelZG - biasZG;
        sample.correctedImu.biasZG = biasZG;
        sample.correctedImu.stationaryBiasApplied = hasStationaryBias;
        sample.correctedImu.closureBiasApplied = false;
    }

    double correctedIntegratedSpeedGs = 0.0;
    for (const ReplaySample &sample : qAsConst(result->samples)) {
        if (sample.hasImu && sample.correctedImu.valid) {
            correctedIntegratedSpeedGs += sample.correctedImu.accelZG * sampleIntervalSeconds;
        }
    }

    const bool needsClosureBias = qAbs(correctedIntegratedSpeedGs) >= kBiasClosureResidualThresholdGs
            && totalIntegrationSeconds > 0.0;
    if (needsClosureBias) {
        const double closureBiasZG = correctedIntegratedSpeedGs / totalIntegrationSeconds;
        for (ReplaySample &sample : result->samples) {
            if (!sample.hasImu || !sample.correctedImu.valid) {
                continue;
            }
            sample.correctedImu.accelZG -= closureBiasZG;
            sample.correctedImu.biasZG += closureBiasZG;
            sample.correctedImu.closureBiasApplied = true;
        }
    }
    const MotionRebuildSummary motionSummary = rebuildDerivedMotion(result, mileageCorrection);

    if (startBias.valid && endBias.valid) {
        appendWarningMessage(&result->warningMessage,
                             QStringLiteral("Z 轴加速度已按首尾静止窗口均值做线性零偏修正"));
    } else if (hasStationaryBias) {
        appendWarningMessage(&result->warningMessage,
                             QStringLiteral("Z 轴加速度已按边界静止窗口均值做常值零偏修正"));
    } else {
        appendWarningMessage(&result->warningMessage,
                             QStringLiteral("未识别到稳定静止窗口，Z 轴加速度已按全程积分闭环估计常值零偏"));
    }

    if (needsClosureBias) {
        appendWarningMessage(&result->warningMessage,
                             QStringLiteral("Z 轴加速度已追加积分闭环偏置回收，消除剩余速度漂移"));
    }

    if (motionSummary.stationaryZeroingApplied) {
        appendWarningMessage(&result->warningMessage,
                             QStringLiteral("解算速度已启用静止清零：修正后 Z 轴加速度连续 %1 s 保持在 +/-%2 g 内时，当前段及往前静止段速度一并归零，且积分速度最小钳制为 0")
                             .arg(kSpeedZeroingStationaryDurationMs / 1000)
                             .arg(QString::number(kSpeedZeroingAccelThresholdG, 'f', 3)));
    }

    if (motionSummary.stationaryZeroingRejectedCount > 0) {
        appendWarningMessage(&result->warningMessage,
                             QStringLiteral("静止清零触发后若距上次实际清零不足 %1 s，则拒绝本次清零并重置清零计数，共拒绝 %2 次")
                             .arg(kSpeedZeroingCooldownMs / 1000)
                             .arg(motionSummary.stationaryZeroingRejectedCount));
    }

    if (mileageCorrection && mileageCorrection->isValid()) {
        appendWarningMessage(&result->warningMessage,
                             QStringLiteral("已加载里程校正表：工作表 %1，共 %2 个站点累计里程")
                             .arg(mileageCorrection->sheetName)
                             .arg(mileageCorrection->stations.size()));
        if (result->mileageCorrectionApplied) {
            appendWarningMessage(&result->warningMessage,
                                 QStringLiteral("已按到站停车事件应用 %1 次里程校正")
                                 .arg(result->mileageCorrectionStopCount));
        } else {
            appendWarningMessage(&result->warningMessage,
                                 QStringLiteral("当前未检测到可用于里程校正的到站停车事件"));
        }
        if (result->mileageCorrectionOverflowCount > 0) {
            appendWarningMessage(&result->warningMessage,
                                 QStringLiteral("到站次数超出校正表范围 %1 次，后续保持原积分结果")
                                 .arg(result->mileageCorrectionOverflowCount));
        }
    }
}

void buildPoseSceneGeometry(ParseResult *result)
{
    if (!result) {
        return;
    }

    result->poseSceneGeometry = PoseSceneGeometry();
    if (result->samples.isEmpty()) {
        return;
    }

    result->poseSceneGeometry.leftRailSections.reserve(result->samples.size());
    result->poseSceneGeometry.rightRailSections.reserve(result->samples.size());

    QVector3D trainPosition(0.0f, kTrainBaseCenterY, 0.0f);

    for (int index = 0; index < result->samples.size(); ++index) {
        const ReplaySample &sample = result->samples.at(index);
        if (index > 0) {
            const qint64 deltaPlaybackMs = sample.playbackMs - result->samples.at(index - 1).playbackMs;
            if (deltaPlaybackMs > 0) {
                const float deltaSeconds = static_cast<float>(deltaPlaybackMs) / 1000.0f;
                const float travelDistance = static_cast<float>(sample.derivedSpeedZGs) * deltaSeconds;
                trainPosition += forwardDirectionForSample(sample) * travelDistance;
            }
        }

        const QMatrix4x4 pose = buildPoseMatrix(sample.poseValid ? static_cast<float>(sample.rollDeg) : 0.0f,
                                                sample.poseValid ? static_cast<float>(sample.pitchDeg) : 0.0f,
                                                sample.poseValid ? static_cast<float>(sample.yawDeg) : 0.0f);

        result->poseSceneGeometry.leftRailSections.push_back(
                    buildSweepSection(trainPosition,
                                      pose,
                                      QVector3D(-kRailCenterOffsetX,
                                                -kTrainHeight * 0.5f - kRailThickness * 0.5f,
                                                0.0f),
                                      kRailWidth * 0.5f,
                                      kRailThickness * 0.5f,
                                      sample.playbackMs));
        result->poseSceneGeometry.rightRailSections.push_back(
                    buildSweepSection(trainPosition,
                                      pose,
                                      QVector3D(kRailCenterOffsetX,
                                                -kTrainHeight * 0.5f - kRailThickness * 0.5f,
                                                0.0f),
                                      kRailWidth * 0.5f,
                                      kRailThickness * 0.5f,
                                      sample.playbackMs));
    }
}
}

bool DataFileParser::loadMileageCorrectionFile(const QString &filePath,
                                               MileageCorrectionTable *table,
                                               QString *errorMessage) const
{
    if (table) {
        *table = MileageCorrectionTable();
    }
    if (errorMessage) {
        errorMessage->clear();
    }

    if (!table) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("里程校正文件输出对象为空。");
        }
        return false;
    }

    QXlsx::Document document(filePath);
    if (!document.isLoadPackage()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("无法读取 xlsx 文件: %1").arg(filePath);
        }
        return false;
    }

    for (const QString &sheetName : document.sheetNames()) {
        if (!document.selectSheet(sheetName)) {
            continue;
        }

        MileageCorrectionTable candidate;
        QString sheetError;
        if (!loadMileageCorrectionSheet(&document, sheetName, &candidate, &sheetError)) {
            if (!sheetError.isEmpty()) {
                if (errorMessage) {
                    *errorMessage = sheetError;
                }
                return false;
            }
            continue;
        }

        candidate.sourceFilePath = filePath;
        *table = candidate;
        return true;
    }

    if (errorMessage) {
        *errorMessage = QStringLiteral("未在 xlsx 中找到包含“车站 / 上行车站中心里程 / 下行车站中心里程 / 上行站间距 / 下行站间距”表头的工作表。");
    }
    return false;
}

ParseResult DataFileParser::parseFile(const QString &filePath,
                                      const MileageCorrectionTable *mileageCorrection) const
{
    ParseResult result;

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        result.errorMessage = QStringLiteral("无法读取文件: %1").arg(file.errorString());
        return result;
    }

    const QByteArray bytes = file.readAll();
    if (bytes.isEmpty()) {
        result.errorMessage = QStringLiteral("文件为空，未读取到任何数据。");
        return result;
    }

    struct ParseScanStats
    {
        int parsedC1 = 0;
        int parsedC2 = 0;
        bool hasFirstC1Timestamp = false;
        bool hasLastC1Timestamp = false;
        qint64 firstC1TimestampMs = -1;
        qint64 lastC1TimestampMs = -1;
        bool sawTimestampRollback = false;
    };

    const ParseScanStats parseScanStats = [this, &bytes]() {
        ParseScanStats stats;
        int scanOffset = 0;
        while (scanOffset + kFrameSize <= bytes.size()) {
            if (!isHeader(bytes, scanOffset)) {
                ++scanOffset;
                continue;
            }

            const quint8 frameType = static_cast<quint8>(bytes.at(scanOffset + 2));
            if (frameType != 0xC1 && frameType != 0xC2) {
                ++scanOffset;
                continue;
            }

            const quint8 storedChecksum = static_cast<quint8>(bytes.at(scanOffset + 37));
            const quint8 calculatedChecksum = checksum(bytes, scanOffset);
            if (storedChecksum != calculatedChecksum) {
                ++scanOffset;
                continue;
            }

            if (frameType == 0xC1) {
                bool timeOk = false;
                const QDateTime timestampUtc = readTimestampUtc(bytes, scanOffset + 5, &timeOk);
                ++stats.parsedC1;
                if (timeOk) {
                    const qint64 currentTimestampMs = timestampUtc.toMSecsSinceEpoch();
                    if (!stats.hasFirstC1Timestamp) {
                        stats.hasFirstC1Timestamp = true;
                        stats.firstC1TimestampMs = currentTimestampMs;
                    }
                    if (stats.hasLastC1Timestamp && currentTimestampMs < stats.lastC1TimestampMs) {
                        stats.sawTimestampRollback = true;
                    }
                    stats.hasLastC1Timestamp = true;
                    stats.lastC1TimestampMs = currentTimestampMs;
                }
            } else {
                ++stats.parsedC2;
            }

            scanOffset += kFrameSize;
        }
        return stats;
    }();

    result.parsedC1 = parseScanStats.parsedC1;
    result.parsedC2 = parseScanStats.parsedC2;
    result.hasFirstC1Timestamp = parseScanStats.hasFirstC1Timestamp;
    result.hasLastC1Timestamp = parseScanStats.hasLastC1Timestamp;
    result.firstC1TimestampMs = parseScanStats.firstC1TimestampMs;
    result.lastC1TimestampMs = parseScanStats.lastC1TimestampMs;

    if (parseScanStats.parsedC2 > 0
            && parseScanStats.hasFirstC1Timestamp
            && parseScanStats.hasLastC1Timestamp
            && parseScanStats.lastC1TimestampMs > parseScanStats.firstC1TimestampMs) {
        const qint64 totalDurationMs = parseScanStats.lastC1TimestampMs - parseScanStats.firstC1TimestampMs;
        result.resolvedSampleIntervalMs = qMax<qint64>(1, qRound(static_cast<double>(totalDurationMs)
                                                                 / static_cast<double>(parseScanStats.parsedC2)));
        result.resolvedSampleIntervalSeconds = static_cast<double>(result.resolvedSampleIntervalMs) / 1000.0;
        appendWarningMessage(&result.warningMessage,
                             QStringLiteral("已根据首尾 C1 时间戳总耗时和 C2 数量计算统一步进 %1 ms")
                             .arg(result.resolvedSampleIntervalMs));
    } else {
        result.resolvedSampleIntervalMs = kDefaultSampleIntervalMs;
        result.resolvedSampleIntervalSeconds = kDefaultSampleIntervalSeconds;
        result.sampleIntervalFallbackUsed = true;
        appendWarningMessage(&result.warningMessage,
                             QStringLiteral("未能根据首尾 C1 时间戳和 C2 数量计算有效步进，已回退为默认 %1 ms")
                             .arg(result.resolvedSampleIntervalMs));
    }
    if (parseScanStats.sawTimestampRollback) {
        appendWarningMessage(&result.warningMessage,
                             QStringLiteral("检测到 C1 时间戳回退，首尾时间统计可能受影响"));
    }

    ReplaySample currentState;
    qint64 playbackMs = 0;
    bool hasTrackOrigin = false;
    double originLatitudeDeg = 0.0;
    double originLongitudeDeg = 0.0;
    double originAltitudeM = 0.0;
    double gyroBiasYDegPerSec = 0.0;
    double gyroBiasZDegPerSec = 0.0;

    int offset = 0;
    while (offset + kFrameSize <= bytes.size()) {
        if (!isHeader(bytes, offset)) {
            ++offset;
            ++result.unknownBytesSkipped;
            continue;
        }

        const quint8 frameType = static_cast<quint8>(bytes.at(offset + 2));
        if (frameType != 0xC1 && frameType != 0xC2) {
            ++offset;
            ++result.unknownBytesSkipped;
            continue;
        }

        const quint8 storedChecksum = static_cast<quint8>(bytes.at(offset + 37));
        const quint8 calculatedChecksum = checksum(bytes, offset);
        if (storedChecksum != calculatedChecksum) {
            ++result.invalidFrames;
            ++offset;
            continue;
        }

        if (frameType == 0xC1) {
            bool timeOk = false;
            const QDateTime timestampUtc = readTimestampUtc(bytes, offset + 5, &timeOk);
            if (timeOk) {
                currentState.hasTimestamp = true;
                currentState.timestampUtc = timestampUtc;
            }

            currentState.hasGps = true;
            currentState.gpsFixMode = static_cast<quint8>(bytes.at(offset + 3));
            currentState.gpsSvNum = static_cast<quint8>(bytes.at(offset + 4));
            currentState.latitudeDeg = static_cast<double>(readInt32LE(bytes, offset + 11)) / 1e7;
            currentState.longitudeDeg = static_cast<double>(readInt32LE(bytes, offset + 15)) / 1e7;
            currentState.altitudeM = static_cast<double>(readInt32LE(bytes, offset + 19)) / 100.0;
            currentState.vxMps = static_cast<double>(readInt32LE(bytes, offset + 23)) / 100.0;
            currentState.vyMps = static_cast<double>(readInt32LE(bytes, offset + 27)) / 100.0;
            currentState.vzMps = static_cast<double>(readInt32LE(bytes, offset + 31)) / 100.0;
            currentState.noiseDbA = static_cast<double>(readUInt16LE(bytes, offset + 35)) / 10.0;

            if (!hasTrackOrigin) {
                hasTrackOrigin = true;
                originLatitudeDeg = currentState.latitudeDeg;
                originLongitudeDeg = currentState.longitudeDeg;
                originAltitudeM = currentState.altitudeM;
            }

            if (hasTrackOrigin) {
                static constexpr double kEarthRadiusM = 6378137.0;
                const double deltaLatRad = degreesToRadians(currentState.latitudeDeg - originLatitudeDeg);
                const double deltaLonRad = degreesToRadians(currentState.longitudeDeg - originLongitudeDeg);
                const double originLatRad = degreesToRadians(originLatitudeDeg);
                const double eastM = deltaLonRad * qCos(originLatRad) * kEarthRadiusM;
                const double northM = deltaLatRad * kEarthRadiusM;

                currentState.relativeEastM = eastM;
                currentState.relativeNorthM = northM;
                currentState.relativeAltitudeM = currentState.altitudeM - originAltitudeM;
                currentState.horizontalDistanceM = qSqrt(eastM * eastM + northM * northM);
            }
        } else {
            currentState.hasImu = true;
            currentState.gyroXDeg = static_cast<double>(readFloatLE(bytes, offset + 4));
            currentState.gyroYDeg = static_cast<double>(readFloatLE(bytes, offset + 8));
            currentState.gyroZDeg = static_cast<double>(readFloatLE(bytes, offset + 12));
            currentState.accelXG = static_cast<double>(readFloatLE(bytes, offset + 16));
            currentState.accelYG = static_cast<double>(readFloatLE(bytes, offset + 20));
            currentState.accelZG = static_cast<double>(readFloatLE(bytes, offset + 24));
            currentState.pressureHpa = static_cast<double>(readInt32LE(bytes, offset + 28)) / 100.0;
            currentState.temperatureC = static_cast<double>(readInt32LE(bytes, offset + 32)) / 10.0;
            currentState.statusFlag = static_cast<quint8>(bytes.at(offset + 36));
            if (currentState.hasImu) {
                const double accelNorm = qSqrt(currentState.accelXG * currentState.accelXG
                                               + currentState.accelYG * currentState.accelYG
                                               + currentState.accelZG * currentState.accelZG);
                const double rollAccelDeg = rollFromAccelDeg(currentState.accelXG, currentState.accelYG);
                const double pitchAccelDeg = pitchFromAccelDeg(currentState.accelXG,
                                                               currentState.accelYG,
                                                               currentState.accelZG);

                double correctedGyroYDegPerSec = currentState.gyroYDeg - gyroBiasYDegPerSec;
                double correctedGyroZDegPerSec = currentState.gyroZDeg - gyroBiasZDegPerSec;

                if (!currentState.poseValid) {
                    currentState.rollDeg = normalizeAngleDeg(rollAccelDeg);
                    currentState.pitchDeg = normalizeAngleDeg(pitchAccelDeg);
                    currentState.yawDeg = 0.0;
                    currentState.poseValid = true;
                } else {
                    const bool usePoseCorrection = shouldApplyPoseCorrection(accelNorm,
                                                                            correctedGyroYDegPerSec,
                                                                            correctedGyroZDegPerSec);
                    if (usePoseCorrection) {
                        gyroBiasYDegPerSec = kGyroBiasUpdateAlpha * gyroBiasYDegPerSec
                                + (1.0 - kGyroBiasUpdateAlpha) * currentState.gyroYDeg;
                        gyroBiasZDegPerSec = kGyroBiasUpdateAlpha * gyroBiasZDegPerSec
                                + (1.0 - kGyroBiasUpdateAlpha) * currentState.gyroZDeg;

                        correctedGyroYDegPerSec = currentState.gyroYDeg - gyroBiasYDegPerSec;
                        correctedGyroZDegPerSec = currentState.gyroZDeg - gyroBiasZDegPerSec;
                    }

                    const double integratedRollDeg = normalizeAngleDeg(currentState.rollDeg
                                                                       + correctedGyroZDegPerSec * result.resolvedSampleIntervalSeconds);
                    const double integratedPitchDeg = normalizeAngleDeg(currentState.pitchDeg
                                                                        + correctedGyroYDegPerSec * result.resolvedSampleIntervalSeconds);
                    const double integratedYawDeg = normalizeAngleDeg(currentState.yawDeg
                                                                      + currentState.gyroXDeg * result.resolvedSampleIntervalSeconds);

                    if (usePoseCorrection) {
                        currentState.rollDeg = normalizeAngleDeg(rollAccelDeg);
                        currentState.pitchDeg = normalizeAngleDeg(pitchAccelDeg);
                    } else {
                        currentState.rollDeg = integratedRollDeg;
                        currentState.pitchDeg = integratedPitchDeg;
                    }
                    currentState.yawDeg = integratedYawDeg;
                }

            }

            ReplaySample sample = currentState;
            if (!sample.hasGps || !hasTrackOrigin) {
                sample.relativeEastM = 0.0;
                sample.relativeNorthM = 0.0;
                sample.relativeAltitudeM = 0.0;
                sample.horizontalDistanceM = 0.0;
            }
            sample.sequence = result.samples.size();
            sample.playbackMs = playbackMs;
            result.samples.append(sample);
            playbackMs += result.resolvedSampleIntervalMs;
        }

        offset += kFrameSize;
    }

    applyCorrectedAccelZ(&result, mileageCorrection);
    buildPoseSceneGeometry(&result);

    if (result.samples.isEmpty()) {
        result.errorMessage = QStringLiteral("未识别到有效的 C1/C2 协议帧。");
    } else if (result.invalidFrames > 0) {
        appendWarningMessage(&result.warningMessage,
                             QStringLiteral("解析完成，但跳过了 %1 个校验失败或损坏的帧。")
                             .arg(result.invalidFrames));
    }

    return result;
}

bool DataFileParser::isHeader(const QByteArray &bytes, int offset)
{
    if (offset + 1 >= bytes.size()) {
        return false;
    }

    const quint8 b0 = static_cast<quint8>(bytes.at(offset));
    const quint8 b1 = static_cast<quint8>(bytes.at(offset + 1));
    return (b0 == 0xEB && b1 == 0x90) || (b0 == 0x90 && b1 == 0xEB);
}

quint8 DataFileParser::checksum(const QByteArray &bytes, int offset)
{
    quint32 sum = 0;
    for (int i = offset + 2; i <= offset + 36; ++i) {
        sum += static_cast<quint8>(bytes.at(i));
    }
    return static_cast<quint8>(sum & 0xFF);
}

qint32 DataFileParser::readInt32LE(const QByteArray &bytes, int offset)
{
    qint32 value = 0;
    value |= static_cast<quint8>(bytes.at(offset));
    value |= static_cast<quint8>(bytes.at(offset + 1)) << 8;
    value |= static_cast<quint8>(bytes.at(offset + 2)) << 16;
    value |= static_cast<quint8>(bytes.at(offset + 3)) << 24;
    return value;
}

quint16 DataFileParser::readUInt16LE(const QByteArray &bytes, int offset)
{
    quint16 value = 0;
    value |= static_cast<quint8>(bytes.at(offset));
    value |= static_cast<quint8>(bytes.at(offset + 1)) << 8;
    return value;
}

float DataFileParser::readFloatLE(const QByteArray &bytes, int offset)
{
    quint32 raw = 0;
    raw |= static_cast<quint8>(bytes.at(offset));
    raw |= static_cast<quint8>(bytes.at(offset + 1)) << 8;
    raw |= static_cast<quint8>(bytes.at(offset + 2)) << 16;
    raw |= static_cast<quint8>(bytes.at(offset + 3)) << 24;

    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(float));
    return value;
}

QDateTime DataFileParser::readTimestampUtc(const QByteArray &bytes, int offset, bool *ok)
{
    const int yearByte = static_cast<quint8>(bytes.at(offset));
    const int year = yearByte < 100 ? 2000 + yearByte : yearByte;
    const int month = static_cast<quint8>(bytes.at(offset + 1));
    const int day = static_cast<quint8>(bytes.at(offset + 2));
    const int hour = static_cast<quint8>(bytes.at(offset + 3));
    const int minute = static_cast<quint8>(bytes.at(offset + 4));
    const int second = static_cast<quint8>(bytes.at(offset + 5));

    const QDate date(year, month, day);
    const QTime time(hour, minute, second);
    const QDateTime timestampUtc(date, time, Qt::UTC);

    const bool valid = isReasonableTimestamp(timestampUtc);
    if (ok) {
        *ok = valid;
    }
    return timestampUtc;
}
