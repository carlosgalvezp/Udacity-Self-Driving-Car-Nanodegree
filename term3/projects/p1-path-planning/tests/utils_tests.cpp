#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "utils.h"

namespace test
{

TEST(ToolsTest, JerkMinimizingTrajectory)
{
    const std::vector<std::vector<double>> test_inputs =
    {
        {0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 1.0},
        {0.0, 10.0, 0.0, 20.0, 15.0, 20.0, 2.0},
        {5.0, 10.0, 2.0, -30.0, -20.0, -4.0, 5.0}
    };

    const std::vector<std::vector<double>> test_answers =
    {
        {0.0, 10.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 10.0, 0.0, 0.0, -0.62499999999999911, 0.31249999999999989},
        {5.0, 10.0, 1.0, -3.0000000000000013, 0.63999999999999968, -0.043199999999999968}
    };

    for (std::size_t i = 0U; i < test_inputs.size(); ++i)
    {
        const std::vector<double>& test_input_i = test_inputs[i];
        const std::vector<double>& test_answer_i = test_answers[i];

        std::vector<double> output;

        const double x0    = test_input_i[0U];
        const double x0_d  = test_input_i[1U];
        const double x0_dd = test_input_i[2U];
        const double xf    = test_input_i[3U];
        const double xf_d  = test_input_i[4U];
        const double xf_dd = test_input_i[5U];
        const double t     = test_input_i[6U];

        generateJerkMinTrajectory(x0, x0_d, x0_dd, xf, xf_d, xf_dd, t, output);

        for (std::size_t j = 0U; j < output.size(); ++j)
        {
            EXPECT_NEAR(test_answer_i[j], output[j], 1.0E-10);
        }
    }
}

}  // namespace test
