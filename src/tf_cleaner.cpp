#include "tf_cleaner.h"

namespace tfCleaner{
    

    tfCleaner::tfCleaner(const int &num){
        std::unique_lock<std::mutex> lock(_mutex);
        // Set initial vars
        tfStorage init;
        init.setIdentity();
        _tfStore.resize(num, init);
    }

    void tfCleaner::addTF(const tfStorage &transform){
        std::unique_lock<std::mutex> lock(_mutex);
        // Rotate left and add to end (FIFO QUEUE)
        // This could all be done with std::queue, but the functions are written to deal with vectors so....
        std::rotate(_tfStore.begin(), _tfStore.begin() + 1, _tfStore.end());

        _tfStore.back() = transform;
    }

    std::vector<int> tfCleaner::getStatisticalInlierIndicesQuat(double std_dev){
        std::unique_lock<std::mutex> lock(_mutex);
        // Return indices of statistical inliers
        std::vector<tf::Quaternion> quatVec;

        for (const auto &t : _tfStore){
            quatVec.push_back(t.getRotation());
        }
        // TODO check length?
        tf::Quaternion aveQuat = getAverageQuaternion(quatVec);
        // First calc variance
        double variance = 0.0;

        std::vector<double> distances;

        for (const auto &q : quatVec){
            double distance = getQuaternionDistance(aveQuat, q); // Save distance to avoid uneccesary further calc
            distances.push_back(distance);

            variance += pow(distance, 2);
        }
        variance /= quatVec.size();

        // STD Dev = sqrt(variance)
        double standardDeviation = sqrt(variance);

        std::vector<int> indices;

        // Get inlier indices
        for (int i = 0; i < distances.size(); i++){
            if (abs(distances[i]) < std_dev * standardDeviation){
                indices.push_back(i);
            }
        }
        return indices;
    }


    std::vector<int> tfCleaner::getStatisticalInlierIndicesTrans(double std_dev){
        std::unique_lock<std::mutex> lock(_mutex);
        // Return indices of statistical inliers
        std::vector<tf::Vector3> transVec;

        for (const auto &t : _tfStore){
            transVec.push_back(t.getOrigin());
        }
        // TODO check length?
        tf::Vector3 aveTrans = getAverageTranslation(transVec);

        // First calc variance
        double variance = 0.0;

        std::vector<double> distances;

        for (const auto &q : transVec){
            double distance = getTranslationDistance(aveTrans, q); // Save distance to avoid uneccesary further calc
            distances.push_back(distance);

            variance += pow(distance, 2);
        }
        variance /= transVec.size();

        // STD Dev = sqrt(variance)
        double standardDeviation = sqrt(variance);

        std::vector<int> indices;

        // Get inlier indices
        for (int i = 0; i < distances.size(); i++){
            if (abs(distances[i]) < std_dev * standardDeviation){
                indices.push_back(i);
            }
        }
        return indices;
    }

    std::vector<tfStorage> tfCleaner::getStatisticalInliers(double std_dev){
        std::unique_lock<std::mutex> lock(_mutex);
        // Get tfs which match both inlier criteria
        lock.unlock();
        std::vector<int> quatInliers = getStatisticalInlierIndicesQuat(std_dev);
        std::vector<int> transInliers = getStatisticalInlierIndicesTrans(std_dev);
        lock.lock();
        std::vector<tfStorage> inliers;

        for (int i = 0; i < _tfStore.size(); i++){
            bool inQuat = std::find(quatInliers.begin(), quatInliers.end(), i) != quatInliers.end();
            bool inTrans = std::find(transInliers.begin(), transInliers.end(), i) != transInliers.end();
            if (inQuat && inTrans){
                inliers.push_back(_tfStore[i]);
            }
        }

        return inliers;
    }
    
    tfStorage tfCleaner::getAverageTF(std::vector<tfStorage> &tfs){
        std::unique_lock<std::mutex> lock(_mutex);
        std::vector<tf::Quaternion> quats;
        std::vector<tf::Vector3> trans;

        for (auto &t : tfs){
            quats.push_back(t.getRotation());
            trans.push_back(t.getOrigin());
        }

        tfStorage ave;
        ave.setRotation(getAverageQuaternion(quats));
        ave.setOrigin(getAverageTranslation(trans));

        return ave;
    }

    tfStorage tfCleaner::getInlierAveTF(double std_dev){
        auto inliers = getStatisticalInliers(std_dev);
        return getAverageTF(inliers);
    }
    /// CLASS tfCleaner



    tf::Quaternion getAverageQuaternion(
    const std::vector<tf::Quaternion>& quaternions, 
    const std::vector<double>& weights)
    {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, quaternions.size());
        Eigen::Vector3d vec;
        for (size_t i = 0; i < quaternions.size(); ++i)
        {
            // Weigh the quaternions according to their associated weight
            tf::Quaternion quat = quaternions[i] * weights[i];
            // Append the weighted Quaternion to a matrix Q.
            Q(0,i) = quat.x();
            Q(1,i) = quat.y();
            Q(2,i) = quat.z();
            Q(3,i) = quat.w();
        }

        // Creat a solver for finding the eigenvectors and eigenvalues
        Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());

        // Find index of maximum (real) Eigenvalue.
        auto eigenvalues = es.eigenvalues();
        size_t max_idx = 0;
        double max_value = eigenvalues[max_idx].real();
        for (size_t i = 1; i < 4; ++i)
        {
            double real = eigenvalues[i].real();
            if (real > max_value)
            {
            max_value = real;
            max_idx = i;
            }
        }

        // Get corresponding Eigenvector, normalize it and return it as the average quat
        auto eigenvector = es.eigenvectors().col(max_idx).normalized();

        tf::Quaternion mean_orientation(
            eigenvector[0].real(),
            eigenvector[1].real(),
            eigenvector[2].real(),
            eigenvector[3].real()
        );

        return mean_orientation.normalized();
    }

    tf::Quaternion getAverageQuaternion(const std::vector<tf::Quaternion>& quaternions){
        // Overload to easily average without input weights
        double _len = quaternions.size();
        std::vector<double> weights(_len, 1.0/_len);

        return getAverageQuaternion(quaternions, weights);
    }

    double getQuaternionDistance(const tf::Quaternion &q1, const tf::Quaternion &q2){
        // Should this be an absolute?
        return (double)q1.angle(q2);
    }

    tf::Vector3 getAverageTranslation(const std::vector<tf::Vector3> &translations){
        // Average translation (vector sum/num)
        double _len = translations.size();
        tf::Vector3 sum;
        for (const tf::Vector3& t : translations){
            sum += t;
        }

        return sum/_len; 
    }

    double getTranslationDistance(const tf::Vector3 &trans1, const tf::Vector3 &trans2){
        // Return Euclidean norm
        tf::Vector3 diff = trans1 - trans2;

        return (double)sqrt(pow(diff.getX(), 2) + pow(diff.getY(), 2) + pow(diff.getZ(), 2));
    }
} //namespace tfCleaner