#include "measurement_synchronizer.h"

namespace se3_lio::synchronizer {

MeasurementSynchronizer::MeasurementSynchronizer() : running_(true), sync_status_(false) {
    sync_thread_ = std::thread(&MeasurementSynchronizer::synchronizeIMULiDARGPS_v2, this);
    sync_thread_.detach();
};

MeasurementSynchronizer::~MeasurementSynchronizer() {
    running_ = false;
    if (sync_thread_.joinable()) {
        sync_thread_.join();
    }
};

void MeasurementSynchronizer::addIMU(const IMU &_imu) {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_queue_.push_back(_imu);
    last_imu_time_ = _imu.header.timestamp;
};

void MeasurementSynchronizer::addLiDAR(const LiDAR &_lidar) {
    std::lock_guard<std::mutex> lock(mutex_);
    lidar_queue_.push_back(_lidar);
};

void MeasurementSynchronizer::addGPS(const GPS &_gps) {
    std::lock_guard<std::mutex> lock(mutex_);
    gps_queue_.push_back(_gps);
};

MeasurementPtr MeasurementSynchronizer::getSyncedMeasurement() {
    MeasurementPtr measurement_ptr(new Measurement());

    if (synced_queue_.empty()) {
        measurement_ptr->is_synced = false;
        return measurement_ptr;
    }

    *measurement_ptr = synced_queue_.front();
    measurement_ptr->is_synced = true;

    mutex_.lock();
    synced_queue_.pop_front();
    mutex_.unlock();

    return measurement_ptr;
};

void MeasurementSynchronizer::synchronizeIMULiDAR() {
    Measurement measurement;
    double lidar_end_time = 0;

    while (running_) {
        {
            // std::lock_guard<std::mutex> lock(mutex_);

            if (imu_queue_.empty() || lidar_queue_.empty()) {
                continue;
            }

            if (!lidar_pushed_) {
                LiDAR lidar_in = lidar_queue_.front();
                lidar_end_time = lidar_in.header.timestamp + lidar_in.points.back().timestamp;
                measurement.lidar = lidar_in;

                lidar_pushed_ = true;
            }

            if (last_imu_time_ < lidar_end_time) {
                continue;
            }

            if (imu_queue_.front().header.timestamp >= lidar_end_time) {
                // No IMU data before the end of LiDAR scan
                lidar_queue_.pop_front();
                lidar_pushed_ = false;
                continue;
            }

            std::vector<IMU> imu_in;
            while (!imu_queue_.empty() && imu_queue_.front().header.timestamp < lidar_end_time) {
                imu_in.push_back(imu_queue_.front());
                imu_queue_.pop_front();
            }

            if (imu_in.empty()) {
                continue;
            }

            measurement.imu = imu_in;

            lidar_queue_.pop_front();
            lidar_pushed_ = false;

            synced_queue_.push_back(measurement);
        }

        // Wait for 1ms to avoid CPU domination
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void MeasurementSynchronizer::synchronizeIMULiDARGPS() {
    while (running_) {
        Measurement measurement;
        double target_end_time = 0.0;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (pause_ || (lidar_queue_.empty() && gps_queue_.empty())) {
                // Nothing to process
                goto sleep_and_continue;
            }

            const bool has_lidar = !lidar_queue_.empty();
            const bool has_gps = !gps_queue_.empty();
            const bool use_lidar_first =
                has_lidar && (!has_gps || lidar_queue_.front().header.timestamp <=
                                              gps_queue_.front().header.timestamp);

            if (use_lidar_first) {
                LiDAR lidar_in = lidar_queue_.front();
                target_end_time = lidar_in.header.timestamp + lidar_in.points.back().timestamp;
                measurement.lidar = lidar_in;
                measurement.gps = GPS();
                lidar_queue_.pop_front();

                std::vector<IMU> imu_in;
                while (!imu_queue_.empty() &&
                       imu_queue_.front().header.timestamp <= target_end_time) {
                    imu_in.push_back(imu_queue_.front());
                    imu_queue_.pop_front();
                }

                if (imu_in.empty()) {
                    // Not enough IMU yet; put lidar back and wait
                    lidar_queue_.push_front(measurement.lidar);
                    goto sleep_and_continue;
                }

                measurement.imu = imu_in;
                measurement.is_synced = true;
            } else {
                // GPS-only measurement
                GPS gps_in = gps_queue_.front();
                measurement.gps = gps_in;
                measurement.lidar = LiDAR();
                measurement.imu.clear();
                measurement.is_synced = true;
                gps_queue_.pop_front();
            }

            synced_queue_.push_back(measurement);
        }

    sleep_and_continue:
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void MeasurementSynchronizer::synchronizeIMULiDARGPS_v2() {
    double prev_lidar_end_time = 0.0;

    while (running_) {
        Measurement measurement;
        double lidar_end_time = 0.0;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (pause_) {
                goto sleep_and_continue;
            }

            if (imu_queue_.empty() || lidar_queue_.empty()) {
                goto sleep_and_continue;
            }

            if (!lidar_pushed_) {
                LiDAR lidar_in = lidar_queue_.front();
                lidar_end_time = lidar_in.header.timestamp + lidar_in.points.back().timestamp;
                measurement.lidar = lidar_in;

                lidar_pushed_ = true;
            }

            if (last_imu_time_ < lidar_end_time) {
                continue;
            }

            std::vector<IMU> imu_in;
            while (!imu_queue_.empty() && imu_queue_.front().header.timestamp < lidar_end_time) {
                imu_in.push_back(imu_queue_.front());
                imu_queue_.pop_front();
            }

            if (imu_in.empty()) {
                continue;
            }

            measurement.imu = imu_in;

            // GPS association
            if (!gps_queue_.empty()) {
                // find the latest GPS before lidar end time
                GPS gps_in = gps_queue_.front();

                while (!gps_queue_.empty() &&
                       gps_queue_.front().header.timestamp < lidar_end_time) {
                    gps_in = gps_queue_.front();
                    gps_queue_.pop_front();
                }

                if (gps_in.header.timestamp < lidar_end_time &&
                    gps_in.header.timestamp > prev_lidar_end_time) {
                    measurement.gps = gps_in;
                } else {
                    measurement.gps = GPS();
                }
            } else {
                measurement.gps = GPS();
            }

            lidar_queue_.pop_front();
            lidar_pushed_ = false;
            prev_lidar_end_time = lidar_end_time;

            synced_queue_.push_back(measurement);
        }

    sleep_and_continue:
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

}  // namespace se3_lio::synchronizer
