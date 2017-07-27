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

TEST(ToolsTest, EvaluatePolynomial)
{
    const std::vector<double> coeffs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    const std::vector<double> inputs = {9.0, 8.0, 7.0, 6.0, 5.0, 4.0};
    const std::vector<double> expected_outputs =
    {
        390277.0,
        219345.0,
        114381.0,
        54121.0,
        22461.0,
        7737.0
    };

    for (std::size_t i = 0U; i < inputs.size(); ++i)
    {
        const double exp = expected_outputs[i];
        const double x = inputs[i];
        EXPECT_NEAR(exp, evaluatePolynomial(coeffs, x), 1.0E-10);
    }
}

TEST(ToolsTest, DifferentiatePolynomial)
{
    // f(x) = 6 + 5x + 4x^2 + 3x^3 + 2x^4 + x^5
    const std::vector<double> coeffs = {6.0, 5.0, 4.0, 3.0, 2.0, 1.0};

    // f'(x) = 5 + 8x + 9x^2 + 8x^3 + 5x^4
    const std::vector<double> diff_coeffs = differentiatePolynomial(coeffs);
    const std::vector<double> exp_diff_coeffs = {5.0, 8.0, 9.0, 8.0, 5.0};

    ASSERT_EQ(exp_diff_coeffs.size(), diff_coeffs.size());
    for (std::size_t i = 0U; i < diff_coeffs.size(); ++i)
    {
        EXPECT_NEAR(exp_diff_coeffs[i], diff_coeffs[i], 1.0E-10);
    }

    // f''(x) = 8 + 18x + 24x^2 + 20x^3
    const std::vector<double> diff_coeffs2 = differentiatePolynomial(diff_coeffs);
    const std::vector<double> exp_diff_coeffs2 = {8.0, 18.0, 24.0, 20.0};

    ASSERT_EQ(exp_diff_coeffs2.size(), diff_coeffs2.size());
    for (std::size_t i = 0U; i < diff_coeffs2.size(); ++i)
    {
        EXPECT_NEAR(exp_diff_coeffs2[i], diff_coeffs2[i], 1.0E-10);
    }

    // f'''(x) = 18 + 48x + 60x^2
    const std::vector<double> diff_coeffs3 = differentiatePolynomial(diff_coeffs2);
    const std::vector<double> exp_diff_coeffs3 = {18.0, 48.0, 60.0};

    ASSERT_EQ(exp_diff_coeffs3.size(), diff_coeffs3.size());
    for (std::size_t i = 0U; i < diff_coeffs3.size(); ++i)
    {
        EXPECT_NEAR(exp_diff_coeffs3[i], diff_coeffs3[i], 1.0E-10);
    }

    // f''''(x) = 48 + 120x
    const std::vector<double> diff_coeffs4 = differentiatePolynomial(diff_coeffs3);
    const std::vector<double> exp_diff_coeffs4 = {48.0, 120.0};

    ASSERT_EQ(exp_diff_coeffs4.size(), diff_coeffs4.size());
    for (std::size_t i = 0U; i < diff_coeffs4.size(); ++i)
    {
        EXPECT_NEAR(exp_diff_coeffs4[i], diff_coeffs4[i], 1.0E-10);
    }
}

}  // namespace test
