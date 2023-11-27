#ifndef TF_CLEANER_H
#define TF_CLEANER_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>

/// \file Functions and class for rejection of outlier tfs using a sliding window, finding averages (usecase here : mainly from Apriltags)
namespace tfCleaner {
    tf::Quaternion getAverageQuaternion(
    const std::vector<tf::Quaternion>& quaternions, 
    const std::vector<double>& weights);

    tf::Quaternion getAverageQuaternion(const std::vector<tf::Quaternion>& quaternions);

    double getQuaternionDistance(const tf::Quaternion &q1, const tf::Quaternion &q2);

    tf::Vector3 getAverageTranslation(const std::vector<tf::Vector3> &translations);

    double getTranslationDistance(const tf::Vector3 &trans1, const tf::Vector3 &trans2);
    
    typedef tf::Transform tfStorage;
    

    class tfCleaner {
        private:
            std::vector<tfStorage> _tfStore;
            std::mutex _mutex;
        public:
            tfCleaner(const int &num); // Sliding window size num
            tfCleaner() : tfCleaner(10) {};

            void addTF(const tfStorage &transform);

            std::vector<int> getStatisticalInlierIndicesQuat(double std_dev = 2.0);

            std::vector<int> getStatisticalInlierIndicesTrans(double std_dev = 2.0);

            std::vector<tfStorage> getStatisticalInliers(double std_dev = 2.0);

            tfStorage getAverageTF(std::vector<tfStorage> &tfs);

            // v This is the key function to clean and return an ave.!
            tfStorage getInlierAveTF(double std_dev = 2.0);
    };

    

} // namespace tfCleaner
#endif //TF_CLEANER_H