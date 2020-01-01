#include "lidar_object_detector/ukf.hpp"

using namespace lidar_object_detector;

/**
* Initializes Unscented Kalman filter
 */
UKF::UKF()
        : num_state_(5)
        , num_lidar_state_(2)
        , num_motion_model_(3)
{
    // initial state vector
    x_merge_ = Eigen::MatrixXd(5, 1);

    // initial state vector
    x_cv_ = Eigen::MatrixXd(5, 1);

    // initial state vector
    x_ctrv_ = Eigen::MatrixXd(5, 1);

    // initial state vector
    x_rm_ = Eigen::MatrixXd(5, 1);

    // initial covariance matrix
    p_merge_ = Eigen::MatrixXd(5, 5);

    // initial covariance matrix
    p_cv_ = Eigen::MatrixXd(5, 5);

    // initial covariance matrix
    p_ctrv_ = Eigen::MatrixXd(5, 5);

    // initial covariance matrix
    p_rm_ = Eigen::MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_cv_ = 1.5;
    std_a_ctrv_ = 1.5;
    std_a_rm_ = 3;
    std_ctrv_yawdd_ = 1.5;
    std_cv_yawdd_ = 1.5;
    std_rm_yawdd_ = 3;

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // time when the state is true, in us
    time_ = 0.0;

    // predicted sigma points matrix
    x_sig_pred_cv_ = Eigen::MatrixXd(num_state_, 2 * num_state_ + 1);

    // predicted sigma points matrix
    x_sig_pred_ctrv_ = Eigen::MatrixXd(num_state_, 2 * num_state_ + 1);

    // predicted sigma points matrix
    x_sig_pred_rm_ = Eigen::MatrixXd(num_state_, 2 * num_state_ + 1);

    // create vector for weights
    weights_c_ = Eigen::VectorXd(2 * num_state_ + 1);
    weights_s_ = Eigen::VectorXd(2 * num_state_ + 1);

    // transition probability
    p1_.push_back(0.9);
    p1_.push_back(0.05);
    p1_.push_back(0.05);

    p2_.push_back(0.05);
    p2_.push_back(0.9);
    p2_.push_back(0.05);

    p3_.push_back(0.05);
    p3_.push_back(0.05);
    p3_.push_back(0.9);

    mode_match_prob_cv2cv_ = 0;
    mode_match_prob_ctrv2cv_ = 0;
    mode_match_prob_rm2cv_ = 0;

    mode_match_prob_cv2ctrv_ = 0;
    mode_match_prob_ctrv2ctrv_ = 0;
    mode_match_prob_rm2ctrv_ = 0;

    mode_match_prob_cv2rm_ = 0;
    mode_match_prob_ctrv2rm_ = 0;
    mode_match_prob_rm2rm_ = 0;

    mode_prob_cv_ = 0.33;
    mode_prob_ctrv_ = 0.33;
    mode_prob_rm_ = 0.33;

    z_pred_cv_ = Eigen::VectorXd(2);
    z_pred_ctrv_ = Eigen::VectorXd(2);
    z_pred_rm_ = Eigen::VectorXd(2);

    s_cv_ = Eigen::MatrixXd(2, 2);
    s_ctrv_ = Eigen::MatrixXd(2, 2);
    s_rm_ = Eigen::MatrixXd(2, 2);

    k_cv_ = Eigen::MatrixXd(5, 2);
    k_ctrv_ = Eigen::MatrixXd(5, 2);
    k_rm_ = Eigen::MatrixXd(5, 2);

    pd_ = 0.9;
    pg_ = 0.99;

    // tracking parameter
    lifetime_ = 0;
    is_static_ = false;

    // bounding box params
    is_stable_ = false;
    object_.dimensions.x = 1.0;
    object_.dimensions.y = 1.0;

    // for static classification
    init_meas_ = Eigen::VectorXd(2);

    x_merge_yaw_ = 0;

    // for raukf
    cv_meas_ = Eigen::VectorXd(2);
    ctrv_meas_ = Eigen::VectorXd(2);
    rm_meas_ = Eigen::VectorXd(2);

    r_cv_ = Eigen::MatrixXd(2, 2);
    r_ctrv_ = Eigen::MatrixXd(2, 2);
    r_rm_ = Eigen::MatrixXd(2, 2);

    q_cv_ = Eigen::MatrixXd(5, 5);
    q_ctrv_ = Eigen::MatrixXd(5, 5);
    q_rm_ = Eigen::MatrixXd(5, 5);
}

double UKF::normalizeAngle(const double angle)
{
    double normalized_angle = angle;
    while (normalized_angle > M_PI)
        normalized_angle -= 2. * M_PI;
    while (normalized_angle < -M_PI)
        normalized_angle += 2. * M_PI;
    return normalized_angle;
}

void UKF::initialize(const Eigen::VectorXd& z, const double timestamp, const int target_id)
{
    ukf_id_ = target_id;

    // first measurement
    x_merge_ << 0, 0, 0, 0, 0.1;

    // init covariance matrix by hardcoding since no clue about initial state covrariance
    p_merge_ << 0.5, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 1;

    // set weights
    // reference from "The Unscented Kalman Filter for Nonlinear Estimation, Eric A. Wan and Rudolph van der Merwe, 2000"
    // alpha = 0.0025, beta = 2, k = 0
    double alpha = 0.0025;
    double beta = 2;
    double k = 0;
    lambda_ = alpha * alpha * (num_state_ + k) - num_state_;
    double weight_s_0 = lambda_ / (lambda_ + num_state_);
    double weight_c_0 = lambda_ / (lambda_ + num_state_) + (1 - alpha * alpha + beta);
    weights_s_(0) = weight_s_0;
    weights_c_(0) = weight_c_0;
    for (int i = 1; i < 2 * num_state_ + 1; i++)
    {  // 2n+1 weights
        double weight = 0.5 / (num_state_ + lambda_);
        weights_s_(i) = weight;
        weights_c_(i) = weight;
    }

    // init timestamp
    time_ = timestamp;

    x_merge_(0) = z(0);
    x_merge_(1) = z(1);

    z_pred_cv_(0) = z(0);
    z_pred_cv_(1) = z(1);

    z_pred_ctrv_(0) = z(0);
    z_pred_ctrv_(1) = z(1);

    z_pred_rm_(0) = z(0);
    z_pred_rm_(1) = z(1);

    x_cv_ = x_ctrv_ = x_rm_ = x_merge_;
    p_cv_ = p_ctrv_ = p_rm_ = p_merge_;

    s_cv_ << 1, 0, 0, 1;
    s_ctrv_ << 1, 0, 0, 1;
    s_rm_ << 1, 0, 0, 1;

    // initialize R covariance
    r_cv_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
    r_ctrv_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
    r_rm_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;

    // init tracking num
    tracking_num_ = 1;
}

void UKF::updateModeProb(const std::vector<double>& lambda_vec)
{
    double cvGauss = lambda_vec[0];
    double ctrvGauss = lambda_vec[1];
    double rmGauss = lambda_vec[2];
    double sumGauss = cvGauss * mode_prob_cv_ + ctrvGauss * mode_prob_ctrv_ + rmGauss * mode_prob_rm_;
    mode_prob_cv_ = (cvGauss * mode_prob_cv_) / sumGauss;
    mode_prob_ctrv_ = (ctrvGauss * mode_prob_ctrv_) / sumGauss;
    mode_prob_rm_ = (rmGauss * mode_prob_rm_) / sumGauss;
    // prevent each prob from becoming 0
    if (fabs(mode_prob_cv_) < 0.0001)
        mode_prob_cv_ = 0.0001;
    if (fabs(mode_prob_ctrv_) < 0.0001)
        mode_prob_ctrv_ = 0.0001;
    if (fabs(mode_prob_rm_) < 0.0001)
        mode_prob_rm_ = 0.0001;
}

void UKF::updateYawWithHighProb()
{
    if (mode_prob_cv_ > mode_prob_ctrv_)
    {
        if (mode_prob_cv_ > mode_prob_rm_)
        {
            x_merge_yaw_ = x_cv_(3);
        }
        else
        {
            x_merge_yaw_ = x_rm_(3);
        }
    }
    else
    {
        if (mode_prob_ctrv_ > mode_prob_rm_)
        {
            x_merge_yaw_ = x_ctrv_(3);
        }
        else
        {
            x_merge_yaw_ = x_rm_(3);
        }
    }
    x_merge_(3) = x_merge_yaw_;
}

void UKF::mergeEstimationAndCovariance()
{
    x_merge_ = mode_prob_cv_ * x_cv_ + mode_prob_ctrv_ * x_ctrv_ + mode_prob_rm_ * x_rm_;
    while (x_merge_(3) > M_PI)
        x_merge_(3) -= 2. * M_PI;
    while (x_merge_(3) < -M_PI)
        x_merge_(3) += 2. * M_PI;

    // not interacting yaw(-pi ~ pi)
    updateYawWithHighProb();

    p_merge_ = mode_prob_cv_ * (p_cv_ + (x_cv_ - x_merge_) * (x_cv_ - x_merge_).transpose()) +
               mode_prob_ctrv_ * (p_ctrv_ + (x_ctrv_ - x_merge_) * (x_ctrv_ - x_merge_).transpose()) +
               mode_prob_rm_ * (p_rm_ + (x_rm_ - x_merge_) * (x_rm_ - x_merge_).transpose());
}

void UKF::mixingProbability()
{
    double sumProb1 = mode_prob_cv_ * p1_[0] + mode_prob_ctrv_ * p2_[0] + mode_prob_rm_ * p3_[0];
    double sumProb2 = mode_prob_cv_ * p1_[1] + mode_prob_ctrv_ * p2_[1] + mode_prob_rm_ * p3_[1];
    double sumProb3 = mode_prob_cv_ * p1_[2] + mode_prob_ctrv_ * p2_[2] + mode_prob_rm_ * p3_[2];
    mode_match_prob_cv2cv_ = mode_prob_cv_ * p1_[0] / sumProb1;
    mode_match_prob_ctrv2cv_ = mode_prob_ctrv_ * p2_[0] / sumProb1;
    mode_match_prob_rm2cv_ = mode_prob_rm_ * p3_[0] / sumProb1;

    mode_match_prob_cv2ctrv_ = mode_prob_cv_ * p1_[1] / sumProb2;
    mode_match_prob_ctrv2ctrv_ = mode_prob_ctrv_ * p2_[1] / sumProb2;
    mode_match_prob_rm2ctrv_ = mode_prob_rm_ * p3_[1] / sumProb2;

    mode_match_prob_cv2rm_ = mode_prob_cv_ * p1_[2] / sumProb3;
    mode_match_prob_ctrv2rm_ = mode_prob_ctrv_ * p2_[2] / sumProb3;
    mode_match_prob_rm2rm_ = mode_prob_rm_ * p3_[2] / sumProb3;
}

void UKF::interaction()
{
    Eigen::MatrixXd x_pre_cv = x_cv_;
    Eigen::MatrixXd x_pre_ctrv = x_ctrv_;
    Eigen::MatrixXd x_pre_rm = x_rm_;
    Eigen::MatrixXd p_pre_cv = p_cv_;
    Eigen::MatrixXd p_pre_ctrv = p_ctrv_;
    Eigen::MatrixXd p_pre_rm = p_rm_;
    x_cv_ = mode_match_prob_cv2cv_ * x_pre_cv + mode_match_prob_ctrv2cv_ * x_pre_ctrv + mode_match_prob_rm2cv_ * x_pre_rm;
    x_ctrv_ = mode_match_prob_cv2ctrv_ * x_pre_cv + mode_match_prob_ctrv2ctrv_ * x_pre_ctrv +
              mode_match_prob_rm2ctrv_ * x_pre_rm;
    x_rm_ = mode_match_prob_cv2rm_ * x_pre_cv + mode_match_prob_ctrv2rm_ * x_pre_ctrv + mode_match_prob_rm2rm_ * x_pre_rm;

    // not interacting yaw(-pi ~ pi)
    x_cv_(3) = x_pre_cv(3);
    x_ctrv_(3) = x_pre_ctrv(3);
    x_rm_(3) = x_pre_rm(3);

    // normalizing angle
    while (x_cv_(3) > M_PI)
        x_cv_(3) -= 2. * M_PI;
    while (x_cv_(3) < -M_PI)
        x_cv_(3) += 2. * M_PI;
    while (x_ctrv_(3) > M_PI)
        x_ctrv_(3) -= 2. * M_PI;
    while (x_ctrv_(3) < -M_PI)
        x_ctrv_(3) += 2. * M_PI;
    while (x_rm_(3) > M_PI)
        x_rm_(3) -= 2. * M_PI;
    while (x_rm_(3) < -M_PI)
        x_rm_(3) += 2. * M_PI;

    p_cv_ = mode_match_prob_cv2cv_ * (p_pre_cv + (x_pre_cv - x_cv_) * (x_pre_cv - x_cv_).transpose()) +
            mode_match_prob_ctrv2cv_ * (p_pre_ctrv + (x_pre_ctrv - x_cv_) * (x_pre_ctrv - x_cv_).transpose()) +
            mode_match_prob_rm2cv_ * (p_pre_rm + (x_pre_rm - x_cv_) * (x_pre_rm - x_cv_).transpose());
    p_ctrv_ = mode_match_prob_cv2ctrv_ * (p_pre_cv + (x_pre_cv - x_ctrv_) * (x_pre_cv - x_ctrv_).transpose()) +
              mode_match_prob_ctrv2ctrv_ * (p_pre_ctrv + (x_pre_ctrv - x_ctrv_) * (x_pre_ctrv - x_ctrv_).transpose()) +
              mode_match_prob_rm2ctrv_ * (p_pre_rm + (x_pre_rm - x_ctrv_) * (x_pre_rm - x_ctrv_).transpose());
    p_rm_ = mode_match_prob_cv2rm_ * (p_pre_cv + (x_pre_cv - x_rm_) * (x_pre_cv - x_rm_).transpose()) +
            mode_match_prob_ctrv2rm_ * (p_pre_ctrv + (x_pre_ctrv - x_rm_) * (x_pre_ctrv - x_rm_).transpose()) +
            mode_match_prob_rm2rm_ * (p_pre_rm + (x_pre_rm - x_rm_) * (x_pre_rm - x_rm_).transpose());
}

void UKF::predictionSUKF(const double dt)
{
    /*****************************************************************************
    *  Init covariance Q if it is necessary
    ****************************************************************************/
    initCovarQs(dt, x_merge_(3));
    /*****************************************************************************
    *  Prediction Motion Model
    ****************************************************************************/
    predictionMotion(dt, MotionModel::CTRV);
    /*****************************************************************************
    *  Prediction Measurement
    ****************************************************************************/
    predictionLidarMeasurement(MotionModel::CTRV, num_lidar_state_);

}

void UKF::predictionIMMUKF(const double dt)
{
    /*****************************************************************************
    *  Init covariance Q if it is needed
    ****************************************************************************/
    initCovarQs(dt, x_merge_(3));
    /*****************************************************************************
    *  IMM Mixing and Interaction
    ****************************************************************************/
    mixingProbability();
    interaction();
    /*****************************************************************************
    *  Prediction Motion Model
    ****************************************************************************/
    predictionMotion(dt, MotionModel::CV);
    predictionMotion(dt, MotionModel::CTRV);
    predictionMotion(dt, MotionModel::RM);
    /*****************************************************************************
    *  Prediction Measurement
    ****************************************************************************/
    predictionLidarMeasurement(MotionModel::CV, num_lidar_state_);
    predictionLidarMeasurement(MotionModel::CTRV, num_lidar_state_);
    predictionLidarMeasurement(MotionModel::RM, num_lidar_state_);
}

void UKF::findMaxZandS(Eigen::VectorXd& max_det_z, Eigen::MatrixXd& max_det_s)
{
    double cv_det = s_cv_.determinant();
    double ctrv_det = s_ctrv_.determinant();
    double rm_det = s_rm_.determinant();

    if (cv_det > ctrv_det)
    {
        if (cv_det > rm_det)
        {
            max_det_z = z_pred_cv_;
            max_det_s = s_cv_;
        }
        else
        {
            max_det_z = z_pred_rm_;
            max_det_s = s_rm_;
        }
    }
    else
    {
        if (ctrv_det > rm_det)
        {
            max_det_z = z_pred_ctrv_;
            max_det_s = s_ctrv_;
        }
        else
        {
            max_det_z = z_pred_rm_;
            max_det_s = s_rm_;
        }
    }
}

void UKF::updateEachMotion(const double detection_probability, const double gate_probability, const double gating_threshold,
                           const std::vector<perception_msgs::DetectedObject>& object_vec,
                           std::vector<double>& lambda_vec)
{
    // calculating association probability
    double num_meas = object_vec.size();
    double b = 2 * num_meas * (1 - detection_probability * gate_probability) / (gating_threshold * detection_probability);

    Eigen::VectorXd max_det_z;
    Eigen::MatrixXd max_det_s;
    findMaxZandS(max_det_z, max_det_s);
    double Vk = M_PI * sqrt(gating_threshold * max_det_s.determinant());

    for (int motion_ind = 0; motion_ind < num_motion_model_; motion_ind++)
    {
        Eigen::MatrixXd x(x_cv_.rows(), x_cv_.cols());
        Eigen::MatrixXd p(p_cv_.rows(), p_cv_.cols());
        bool is_direction_available = false;
        int num_meas_state = 0;
        Eigen::VectorXd z_pred;
        Eigen::MatrixXd s_pred;
        Eigen::MatrixXd kalman_gain;
        Eigen::VectorXd likely_meas;
        double e_sum = 0;
        std::vector<double> e_vec;
        std::vector<Eigen::VectorXd> diff_vec;
        std::vector<Eigen::VectorXd> meas_vec;

        if (motion_ind == MotionModel::CV)
        {
            x = x_cv_;
            p = p_cv_;
            num_meas_state = num_lidar_state_;
            z_pred = Eigen::VectorXd(num_meas_state);
            s_pred = Eigen::MatrixXd(num_meas_state, num_meas_state);
            kalman_gain = Eigen::MatrixXd(num_state_, num_meas_state);
            z_pred = z_pred_cv_;
            s_pred = s_cv_;
            kalman_gain = k_cv_;
        }
        else if (motion_ind == MotionModel::CTRV)
        {
            x = x_ctrv_;
            p = p_ctrv_;
            num_meas_state = num_lidar_state_;
            z_pred = Eigen::VectorXd(num_meas_state);
            s_pred = Eigen::MatrixXd(num_meas_state, num_meas_state);
            kalman_gain = Eigen::MatrixXd(num_state_, num_meas_state);
            z_pred = z_pred_ctrv_;
            s_pred = s_ctrv_;
            kalman_gain = k_ctrv_;
        }
        else
        {
            x = x_rm_;
            p = p_rm_;
            num_meas_state = num_lidar_state_;
            z_pred = Eigen::VectorXd(num_meas_state);
            s_pred = Eigen::MatrixXd(num_meas_state, num_meas_state);
            kalman_gain = Eigen::MatrixXd(num_state_, num_meas_state);
            z_pred = z_pred_rm_;
            s_pred = s_rm_;
            kalman_gain = k_rm_;
        }

        for (size_t i = 0; i < num_meas; i++)
        {
            Eigen::VectorXd meas = Eigen::VectorXd(num_meas_state);
            meas(0) = object_vec[i].pose.position.x;
            meas(1) = object_vec[i].pose.position.y;
            if (is_direction_available)
                meas(2) = getYawFromOrient(object_vec[i].pose.orientation);
            meas_vec.push_back(meas);
            Eigen::VectorXd diff = meas - z_pred;
            diff_vec.push_back(diff);
            double e = exp(-0.5 * diff.transpose() * s_pred.inverse() * diff);
            e_vec.push_back(e);
            e_sum += e;
        }
        double beta_zero = b / (b + e_sum);

        std::vector<double> beta_vec;

        if (num_meas != 0)
        {
            std::vector<double>::iterator max_iter = std::max_element(e_vec.begin(), e_vec.end());
            int max_ind = std::distance(e_vec.begin(), max_iter);
            likely_meas = meas_vec[max_ind];
        }

        for (size_t i = 0; i < num_meas; i++)
        {
            double temp = e_vec[i] / (b + e_sum);
            beta_vec.push_back(temp);
        }
        Eigen::VectorXd sigma_x;
        sigma_x.setZero(num_meas_state);

        for (size_t i = 0; i < num_meas; i++)
        {
            sigma_x += beta_vec[i] * diff_vec[i];
        }

        Eigen::MatrixXd sigma_p;
        sigma_p.setZero(num_meas_state, num_meas_state);

        for (size_t i = 0; i < num_meas; i++)
        {
            sigma_p += (beta_vec[i] * diff_vec[i] * diff_vec[i].transpose() - sigma_x * sigma_x.transpose());
        }

        // update x and P
        Eigen::MatrixXd updated_x(x_cv_.rows(), x_cv_.cols());
        updated_x = x + kalman_gain * sigma_x;

        updated_x(3) = normalizeAngle(updated_x(3));

        Eigen::MatrixXd updated_p(p_cv_.rows(), p_cv_.cols());
        if (num_meas != 0)
        {
            updated_p = beta_zero * p + (1 - beta_zero) * (p - kalman_gain * s_pred * kalman_gain.transpose()) +
                        kalman_gain * sigma_p * kalman_gain.transpose();
        }
        else
        {
            updated_p = p - kalman_gain * s_pred * kalman_gain.transpose();
        }

        double lambda;
        if (num_meas != 0)
        {
            lambda =
                    (1 - gate_probability * detection_probability) / pow(Vk, num_meas) +
                    detection_probability * pow(Vk, 1 - num_meas) * e_sum / (num_meas * sqrt(2 * M_PI * s_pred.determinant()));
        }
        else
        {
            lambda = (1 - gate_probability * detection_probability);
        }

        lambda_vec.push_back(lambda);

        if (motion_ind == MotionModel::CV)
        {
            x_cv_ = updated_x;
            p_cv_ = updated_p;
        }
        else if (motion_ind == MotionModel::CTRV)
        {
            x_ctrv_ = updated_x;
            p_ctrv_ = updated_p;
        }
        else
        {
            x_rm_ = updated_x;
            p_rm_ = updated_p;
        }
    }
}

void UKF::updateMeasurementForCTRV(const std::vector<perception_msgs::DetectedObject>& object_vec)
{
    std::vector<double> e_ctrv_vec;
    std::vector<Eigen::VectorXd> meas_vec;
    for (auto const& object : object_vec)
    {
        Eigen::VectorXd meas;
        meas = Eigen::VectorXd(num_lidar_state_);
        meas << object.pose.position.x, object.pose.position.y;
        meas_vec.push_back(meas);
        Eigen::VectorXd diff_ctrv = meas - z_pred_ctrv_;
        double e_ctrv = exp(-0.5 * diff_ctrv.transpose() * s_ctrv_.inverse() * diff_ctrv);
        e_ctrv_vec.push_back(e_ctrv);
    }
    std::vector<double>::iterator max_ctrv_iter = std::max_element(e_ctrv_vec.begin(), e_ctrv_vec.end());
    int max_ctrv_ind = std::distance(e_ctrv_vec.begin(), max_ctrv_iter);
    ctrv_meas_ = meas_vec[max_ctrv_ind];
}

void UKF::uppateForCTRV()
{
    Eigen::VectorXd x = x_ctrv_.col(0);

    x_ctrv_.col(0) = x + k_ctrv_ * (ctrv_meas_ - z_pred_ctrv_);
    p_ctrv_ = p_ctrv_ - k_ctrv_ * s_ctrv_ * k_ctrv_.transpose();
    x_merge_.col(0) = x_ctrv_.col(0);
}

void UKF::updateSUKF(const std::vector<perception_msgs::DetectedObject>& object_vec)
{
    // Applying this skip process only to updateSUKF
    // since updateIMMUKF update covariance even if there is no measurement.
    if (object_vec.empty())
    {
        return;
    }
    updateKalmanGain(MotionModel::CTRV);
    updateMeasurementForCTRV(object_vec);
    uppateForCTRV();
}

void UKF::updateIMMUKF(const double detection_probability, const double gate_probability, const double gating_threshold,
                       const std::vector<perception_msgs::DetectedObject>& object_vec)
{
    /*****************************************************************************
    *  IMM Update
    ****************************************************************************/
    // update kalman gain
    updateKalmanGain(MotionModel::CV);
    updateKalmanGain(MotionModel::CTRV);
    updateKalmanGain(MotionModel::RM);

    // update state varibale x and state covariance p
    std::vector<double> lambda_vec;
    updateEachMotion(detection_probability, gate_probability, gating_threshold, object_vec, lambda_vec);
    /*****************************************************************************
    *  IMM Merge Step
    ****************************************************************************/
    updateModeProb(lambda_vec);
    mergeEstimationAndCovariance();
}

void UKF::ctrv(const double p_x, const double p_y, const double v, const double yaw, const double yawd,
               const double delta_t, std::vector<double>& state)
{
    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001)
    {
        px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else
    {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }
    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    while (yaw_p > M_PI)
        yaw_p -= 2. * M_PI;
    while (yaw_p < -M_PI)
        yaw_p += 2. * M_PI;

    state[0] = px_p;
    state[1] = py_p;
    state[2] = v_p;
    state[3] = yaw_p;
    state[4] = yawd_p;
}

void UKF::cv(const double p_x, const double p_y, const double v, const double yaw, const double yawd,
             const double delta_t, std::vector<double>& state)
{
    // Reference: Bayesian Environment Representation, Prediction, and Criticality Assessment for Driver Assistance
    // Systems, 2016
    double px_p = p_x + v * cos(yaw) * delta_t;
    double py_p = p_y + v * sin(yaw) * delta_t;
    double v_p = v;
    double yaw_p = yaw;
    double yawd_p = 0;

    state[0] = px_p;
    state[1] = py_p;
    state[2] = v_p;
    state[3] = yaw_p;
    state[4] = yawd_p;
}

void UKF::randomMotion(const double p_x, const double p_y, const double v, const double yaw, const double yawd,
                       const double delta_t, std::vector<double>& state)
{
    // Reference: Bayesian Environment Representation, Prediction, and Criticality Assessment for Driver Assistance
    // Systems, 2016
    double px_p = p_x;
    double py_p = p_y;
    double v_p = 0.0;
    double yaw_p = yaw;
    double yawd_p = 0;

    state[0] = px_p;
    state[1] = py_p;
    state[2] = v_p;
    state[3] = yaw_p;
    state[4] = yawd_p;
}

void UKF::initCovarQs(const double dt, const double yaw)
{
    if (tracking_num_ != TrackingState::Init)
    {
        return;
    }
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    double cos_2_yaw = cos(yaw) * cos(yaw);
    double sin_2_yaw = sin(yaw) * sin(yaw);
    double cos_sin = cos_yaw * sin_yaw;

    double cv_var_a = std_a_cv_ * std_a_cv_;
    double cv_var_yawdd = std_cv_yawdd_ * std_cv_yawdd_;

    double ctrv_var_a = std_a_ctrv_ * std_a_ctrv_;
    double ctrv_var_yawdd = std_ctrv_yawdd_ * std_ctrv_yawdd_;

    double rm_var_a = std_a_rm_ * std_a_rm_;
    double rm_var_yawdd = std_rm_yawdd_ * std_rm_yawdd_;

    q_cv_ << 0.5 * 0.5 * dt_4 * cos_2_yaw * cv_var_a, 0.5 * 0.5 * dt_4 * cos_sin * cv_var_a,
            0.5 * dt_3 * cos_yaw * cv_var_a, 0, 0, 0.5 * 0.5 * dt_4 * cos_sin * cv_var_a,
            0.5 * 0.5 * dt_4 * sin_2_yaw * cv_var_a, 0.5 * dt_3 * sin_yaw * cv_var_a, 0, 0, 0.5 * dt_3 * cos_yaw * cv_var_a,
            0.5 * dt_3 * sin_yaw * cv_var_a, dt_2 * cv_var_a, 0, 0, 0, 0, 0, 0.5 * 0.5 * dt_4 * cv_var_yawdd,
            0.5 * dt_3 * cv_var_yawdd, 0, 0, 0, 0.5 * dt_3 * cv_var_yawdd, dt_2 * cv_var_yawdd;
    q_ctrv_ << 0.5 * 0.5 * dt_4 * cos_2_yaw * ctrv_var_a, 0.5 * 0.5 * dt_4 * cos_sin * ctrv_var_a,
            0.5 * dt_3 * cos_yaw * ctrv_var_a, 0, 0, 0.5 * 0.5 * dt_4 * cos_sin * ctrv_var_a,
            0.5 * 0.5 * dt_4 * sin_2_yaw * ctrv_var_a, 0.5 * dt_3 * sin_yaw * ctrv_var_a, 0, 0,
            0.5 * dt_3 * cos_yaw * ctrv_var_a, 0.5 * dt_3 * sin_yaw * ctrv_var_a, dt_2 * ctrv_var_a, 0, 0, 0, 0, 0,
            0.5 * 0.5 * dt_4 * ctrv_var_yawdd, 0.5 * dt_3 * ctrv_var_yawdd, 0, 0, 0, 0.5 * dt_3 * ctrv_var_yawdd,
            dt_2 * ctrv_var_yawdd;
    q_rm_ << 0.5 * 0.5 * dt_4 * cos_2_yaw * rm_var_a, 0.5 * 0.5 * dt_4 * cos_sin * rm_var_a,
            0.5 * dt_3 * cos_yaw * rm_var_a, 0, 0, 0.5 * 0.5 * dt_4 * cos_sin * rm_var_a,
            0.5 * 0.5 * dt_4 * sin_2_yaw * rm_var_a, 0.5 * dt_3 * sin_yaw * rm_var_a, 0, 0, 0.5 * dt_3 * cos_yaw * rm_var_a,
            0.5 * dt_3 * sin_yaw * rm_var_a, dt_2 * rm_var_a, 0, 0, 0, 0, 0, 0.5 * 0.5 * dt_4 * rm_var_yawdd,
            0.5 * dt_3 * rm_var_yawdd, 0, 0, 0, 0.5 * dt_3 * rm_var_yawdd, dt_2 * rm_var_yawdd;
}

void UKF::predictionMotion(const double delta_t, const int model_ind)
{
    /*****************************************************************************
   *  Initialize model parameters
   ****************************************************************************/
    Eigen::MatrixXd x(x_cv_.rows(), 1);
    Eigen::MatrixXd p(p_cv_.rows(), p_cv_.cols());
    Eigen::MatrixXd q(p_cv_.rows(), p_cv_.cols());
    Eigen::MatrixXd x_sig_pred(x_sig_pred_cv_.rows(), x_sig_pred_cv_.cols());
    if (model_ind == MotionModel::CV)
    {
        x = x_cv_.col(0);
        p = p_cv_;
        q = q_cv_;
        x_sig_pred = x_sig_pred_cv_;
    }
    else if (model_ind == MotionModel::CTRV)
    {
        x = x_ctrv_.col(0);
        p = p_ctrv_;
        q = q_ctrv_;
        x_sig_pred = x_sig_pred_ctrv_;
    }
    else
    {
        x = x_rm_.col(0);
        p = p_rm_;
        q = q_rm_;
        x_sig_pred = x_sig_pred_rm_;
    }

    /*****************************************************************************
    *  Create Sigma Points
    ****************************************************************************/

    Eigen::MatrixXd x_sig = Eigen::MatrixXd(num_state_, 2 * num_state_ + 1);

    // create square root matrix
    Eigen::MatrixXd L = p.llt().matrixL();

    // create augmented sigma points
    x_sig.col(0) = x;
    for (int i = 0; i < num_state_; i++)
    {
        Eigen::VectorXd pred1 = x + sqrt(lambda_ + num_state_) * L.col(i);
        Eigen::VectorXd pred2 = x - sqrt(lambda_ + num_state_) * L.col(i);

        while (pred1(3) > M_PI)
            pred1(3) -= 2. * M_PI;
        while (pred1(3) < -M_PI)
            pred1(3) += 2. * M_PI;

        while (pred2(3) > M_PI)
            pred2(3) -= 2. * M_PI;
        while (pred2(3) < -M_PI)
            pred2(3) += 2. * M_PI;

        x_sig.col(i + 1) = pred1;
        x_sig.col(i + 1 + num_state_) = pred2;
    }

    /*****************************************************************************
    *  Predict Sigma Points
    ****************************************************************************/
    // predict sigma points
    for (int i = 0; i < 2 * num_state_ + 1; i++)
    {
        // extract values for better readability
        double p_x = x_sig(0, i);
        double p_y = x_sig(1, i);
        double v = x_sig(2, i);
        double yaw = x_sig(3, i);
        double yawd = x_sig(4, i);

        std::vector<double> state(5);
        if (model_ind == MotionModel::CV)
            cv(p_x, p_y, v, yaw, yawd, delta_t, state);
        else if (model_ind == MotionModel::CTRV)
            ctrv(p_x, p_y, v, yaw, yawd, delta_t, state);
        else
            randomMotion(p_x, p_y, v, yaw, yawd, delta_t, state);

        // write predicted sigma point into right column
        x_sig_pred(0, i) = state[0];
        x_sig_pred(1, i) = state[1];
        x_sig_pred(2, i) = state[2];
        x_sig_pred(3, i) = state[3];
        x_sig_pred(4, i) = state[4];
    }

    /*****************************************************************************
    *  Convert Predicted Sigma Points to Mean/Covariance
    ****************************************************************************/
    // predicted state mean
    x.fill(0.0);
    for (int i = 0; i < 2 * num_state_ + 1; i++)
    {  // iterate over sigma points
        x = x + weights_s_(i) * x_sig_pred.col(i);
    }

    while (x(3) > M_PI)
        x(3) -= 2. * M_PI;
    while (x(3) < -M_PI)
        x(3) += 2. * M_PI;
    // predicted state covariance matrix
    p.fill(0.0);
    for (int i = 0; i < 2 * num_state_ + 1; i++)
    {  // iterate over sigma points
        // state difference
        Eigen::VectorXd x_diff = x_sig_pred.col(i) - x;
        // angle normalization
        while (x_diff(3) > M_PI)
            x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI)
            x_diff(3) += 2. * M_PI;
        p = p + weights_c_(i) * x_diff * x_diff.transpose();
    }

    p = p + q;

    /*****************************************************************************
    *  Update model parameters
    ****************************************************************************/
    if (model_ind == MotionModel::CV)
    {
        x_cv_.col(0) = x;
        p_cv_ = p;
        x_sig_pred_cv_ = x_sig_pred;
    }
    else if (model_ind == MotionModel::CTRV)
    {
        x_ctrv_.col(0) = x;
        p_ctrv_ = p;
        x_sig_pred_ctrv_ = x_sig_pred;
    }
    else
    {
        x_rm_.col(0) = x;
        p_rm_ = p;
        x_sig_pred_rm_ = x_sig_pred;
    }
}

void UKF::updateKalmanGain(const int motion_ind)
{
    Eigen::VectorXd x(x_cv_.rows());
    Eigen::MatrixXd x_sig_pred(x_sig_pred_cv_.rows(), x_sig_pred_cv_.cols());
    Eigen::VectorXd z_pred;
    Eigen::MatrixXd s_pred;
    int num_meas_state = 0;
    if (motion_ind == MotionModel::CV)
    {
        x = x_cv_.col(0);
        x_sig_pred = x_sig_pred_cv_;
        num_meas_state = num_lidar_state_;
        z_pred = Eigen::VectorXd(num_meas_state);
        z_pred = z_pred_cv_;
        s_pred = Eigen::MatrixXd(num_meas_state, num_meas_state);
        s_pred = s_cv_;
    }
    else if (motion_ind == MotionModel::CTRV)
    {
        x = x_ctrv_.col(0);
        x_sig_pred = x_sig_pred_ctrv_;
        num_meas_state = num_lidar_state_;
        z_pred = Eigen::VectorXd(num_meas_state);
        z_pred = z_pred_ctrv_;
        s_pred = Eigen::MatrixXd(num_meas_state, num_meas_state);
        s_pred = s_ctrv_;
    }
    else
    {
        x = x_rm_.col(0);
        x_sig_pred = x_sig_pred_rm_;
        num_meas_state = num_lidar_state_;
        z_pred = Eigen::VectorXd(num_meas_state);
        z_pred = z_pred_rm_;
        s_pred = Eigen::MatrixXd(num_meas_state, num_meas_state);
        s_pred = s_rm_;
    }

    Eigen::MatrixXd cross_covariance = Eigen::MatrixXd(num_state_, num_meas_state);
    cross_covariance.fill(0.0);
    for (int i = 0; i < 2 * num_state_ + 1; i++)
    {
        Eigen::VectorXd z_sig_point(num_meas_state);
        z_sig_point << x_sig_pred(0, i), x_sig_pred(1, i);
        Eigen::VectorXd z_diff = z_sig_point - z_pred;
        Eigen::VectorXd x_diff = x_sig_pred.col(i) - x;

        x_diff(3) = normalizeAngle(x_diff(3));

        cross_covariance = cross_covariance + weights_c_(i) * x_diff * z_diff.transpose();
    }

    Eigen::MatrixXd kalman_gain = cross_covariance * s_pred.inverse();

    if (motion_ind == MotionModel::CV)
    {
        k_cv_ = kalman_gain;
    }
    else if (motion_ind == MotionModel::CTRV)
    {
        k_ctrv_ = kalman_gain;
    }
    else
    {
        k_rm_ = kalman_gain;
    }
}

void UKF::predictionLidarMeasurement(const int motion_ind, const int num_meas_state)
{
    Eigen::MatrixXd x_sig_pred(x_sig_pred_cv_.rows(), x_sig_pred_cv_.cols());
    Eigen::MatrixXd covariance_r(num_meas_state, num_meas_state);
    if (motion_ind == MotionModel::CV)
    {
        x_sig_pred = x_sig_pred_cv_;
        covariance_r = r_cv_;
    }
    else if (motion_ind == MotionModel::CTRV)
    {
        x_sig_pred = x_sig_pred_ctrv_;
        covariance_r = r_ctrv_;
    }
    else
    {
        x_sig_pred = x_sig_pred_rm_;
        covariance_r = r_rm_;
    }

    Eigen::MatrixXd z_sig = Eigen::MatrixXd(num_meas_state, 2 * num_state_ + 1);

    for (int i = 0; i < 2 * num_state_ + 1; i++)
    {
        double p_x = x_sig_pred(0, i);
        double p_y = x_sig_pred(1, i);

        z_sig(0, i) = p_x;
        z_sig(1, i) = p_y;
    }

    Eigen::VectorXd z_pred = Eigen::VectorXd(num_meas_state);
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * num_state_ + 1; i++)
    {
        z_pred = z_pred + weights_s_(i) * z_sig.col(i);
    }

    Eigen::MatrixXd s_pred = Eigen::MatrixXd(num_meas_state, num_meas_state);
    s_pred.fill(0.0);
    for (int i = 0; i < 2 * num_state_ + 1; i++)
    {
        Eigen::VectorXd z_diff = z_sig.col(i) - z_pred;
        s_pred = s_pred + weights_c_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    s_pred += covariance_r;

    if (motion_ind == MotionModel::CV)
    {
        z_pred_cv_ = z_pred;
        s_cv_ = s_pred;
    }
    else if (motion_ind == MotionModel::CTRV)
    {
        z_pred_ctrv_ = z_pred;
        s_ctrv_ = s_pred;
    }
    else
    {
        z_pred_rm_ = z_pred;
        s_rm_ = s_pred;
    }
}

void UKF::prediction(const bool use_sukf, const double dt)
{
    if (use_sukf)
    {
        predictionSUKF(dt);
    }
    else
    {
        predictionIMMUKF(dt);
    }
}

void UKF::update(const bool use_sukf, const double detection_probability, const double gate_probability,
                 const double gating_threshold, const std::vector<perception_msgs::DetectedObject>& object_vec)
{
    if (use_sukf)
    {
        updateSUKF(object_vec);
    }
    else
    {
        updateIMMUKF(detection_probability, gate_probability, gating_threshold, object_vec);
    }
}

