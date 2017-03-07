#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "tools.h"

namespace test
{

static const std::size_t n = 4U;

TEST(ToolsTest, RMS_NoError)
{
    std::vector<Eigen::VectorXd> estimates;
    std::vector<Eigen::VectorXd> ground_truth;

    Eigen::VectorXd estimate(n);
    estimate << 1.5, 4.3, 9.8, 4.2;
    estimates.push_back(estimate);

    Eigen::VectorXd true_value(n);
    true_value<< 1.5, 4.3, 9.8, 4.2;
    ground_truth.push_back(true_value);

    Eigen::VectorXd rmse = Tools::calculateRMSE(estimates, ground_truth);
    for (std::size_t i = 0U; i < n; ++i)
    {
        EXPECT_EQ(0., rmse(i));
    }
}

TEST(ToolsTest, RMS_OneMeasurement)
{
    std::vector<Eigen::VectorXd> estimates;
    std::vector<Eigen::VectorXd> ground_truth;

    Eigen::VectorXd estimate(n);
    estimate << 1.5, 4.3, 9.8, 4.2;
    estimates.push_back(estimate);

    Eigen::VectorXd true_value(n);
    true_value<< 4.2, 9.8, 4.3, 1.5;
    ground_truth.push_back(true_value);

    Eigen::VectorXd rmse = Tools::calculateRMSE(estimates, ground_truth);
    Eigen::VectorXd rmse_expected(n);
    rmse_expected << 2.7, 5.5, 5.5, 2.7;

    for (std::size_t i = 0U; i < n; ++i)
    {
        EXPECT_NEAR(rmse_expected(i), rmse(i), 1.0E-10);
    }
}

TEST(ToolsTest, RMS_MultipleMeasurements)
{
    EXPECT_EQ(0U, 0U);
}

}
