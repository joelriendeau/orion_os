#pragma once

#include <list>
#include "PatchedEigen/patched_eigen.hpp"
#include "covariance.hpp"

namespace stats {

namespace integration {

    // integrate a set of values and their associated variances into a resulting value and variance.
    // the result will be weighted into a more accurate version. this algorithm is detailed in
    // "Combining Error Ellipses" by John E. Davis - cxc.harvard.edu/csc/memos/files/Davis_ellipse.pdf
    template<class coords_iterator_t, class vars_iterator_t>
    void integrate_scalar_davis(const coords_iterator_t& values_begin, const coords_iterator_t& values_end,
        const vars_iterator_t& vars_begin, const vars_iterator_t& vars_end,
        typename coords_iterator_t::value_type& value_result,
        typename vars_iterator_t::value_type& var_result)
    {
        value_result = 0.;
        var_result = 0.;
        vars_iterator_t it_var = vars_begin;
        coords_iterator_t it_value = values_begin;
        for (;it_var != vars_end; ++it_var, ++it_value)
        {
            typename vars_iterator_t::value_type inv = 1 / *it_var;
            value_result += inv * *it_value;
            var_result += inv;
        }
        var_result = 1. / var_result;
        value_result = var_result * value_result;
    }

    // vector version of integrate_scalar_davis. iterator should be over Eigen::Matrix objects
    template<class coords_iterator_t, class covars_iterator_t>
    void integrate_vector_davis(const coords_iterator_t& values_begin, const coords_iterator_t& values_end,
        const covars_iterator_t& covars_begin, const covars_iterator_t& covars_end,
        typename coords_iterator_t::value_type& value_result,
        typename covars_iterator_t::value_type& covar_result)
    {
        value_result = coords_iterator_t::value_type::Zero();
        covar_result = covars_iterator_t::value_type::Zero();
        covars_iterator_t it_covar = covars_begin;
        coords_iterator_t it_value = values_begin;
        for (;it_covar != covars_end; ++it_covar, ++it_value)
        {
            typename covars_iterator_t::value_type inv = it_covar->inverse();
            value_result += inv * *it_value;
            covar_result += inv;
        }
        covar_result = covar_result.inverse().eval();
        value_result = covar_result * value_result;
    }

    // 2-elements-and-update version of integrate_vector_davis.
    template<class coords_vector_t, class covars_matrix_t>
    void integrate_update_davis(const coords_vector_t& new_coord, coords_vector_t& update_coord,
        const covars_matrix_t& new_covar, covars_matrix_t& update_covar)
    {
        covars_matrix_t new_inv = new_covar.inverse();
        covars_matrix_t upd_inv = update_covar.inverse();
        update_coord = upd_inv * update_coord + new_inv * new_coord;
        update_covar = (upd_inv + new_inv).inverse();
        update_coord = update_covar * update_coord;
    }

    // mathematically optimized version of integrate_vector_davis - may run faster or slower depending on matrix dimensions and CPU type
    template<class coords_iterator_t, class covars_iterator_t>
    void integrate_vector_ellipsis(const coords_iterator_t& values_begin, const coords_iterator_t& values_end,
        const covars_iterator_t& covars_begin, const covars_iterator_t& covars_end,
        typename coords_iterator_t::value_type& value_result,
        typename covars_iterator_t::value_type& covar_result)
    {
        value_result = *values_begin;
        covar_result = *covars_begin;
        coords_iterator_t it_value = values_begin; ++it_value;
        covars_iterator_t it_covar = covars_begin; ++it_covar;
        for (;it_covar != covars_end; ++it_covar, ++it_value)
        {
            typename covars_iterator_t::value_type covar_inv = covar_result + *it_covar;
            covar_inv = covar_inv.inverse().eval();
            value_result = (*it_covar * covar_inv * value_result + covar_result * covar_inv * *it_value);
            covar_result = covar_result * covar_inv * *it_covar;
        }
    }

    // controller for use with raw data. it will monitor the integration process for possible problems.
    class integrator
    {
    public:
        enum status
        {
            ok = 0,
            new_point_too_far = 1, // last added point is very far from accumulated value
            in_motion = 2,         // analysis of all points in integration shows unstable rover.
            poor_quality = 3,      // last added point has very poor variance compared to other data in the integral.
        };

        integrator(float multipath_period) : period(multipath_period)
        {
            clear();
        }

        void clear()
        {
            continuous_samples = 0;
        }

        status add_coordinate(float sample_time_seconds, const Eigen::Vector3d& position, const Eigen::Matrix3d& covariance)
        {
            if (continuous_samples == 0)
            {
                integrated_pos = position;
                integrated_var = covariance;
                continuous_samples = 1;
                last_sample_time = sample_time_seconds;
                return ok;
            }
            else
            {
                double distance = (position - integrated_pos).norm();
                double sep_99;
                stats::covariance::sep_3d_99(integrated_var, sep_99);
                // TODO : improved distance test
                // - separate horizontal from vertical tests, as variances are sometimes very different
                // - use better accuracy algorithms
                if (distance > sep_99) // is the new point outside the 99% probability sphere?
                    return new_point_too_far;
            }

            // TODO : bad new variance test
            // TODO : unstable test (uses all recorded points and looks for higher std than what predicted by covariance)

            float delta = sample_time_seconds - last_sample_time;
            if (delta == 0)
                return ok;
            last_sample_time = sample_time_seconds;

            double alpha = (delta < period) ? period / delta : 1.f;
            double integrated_sum = integrated_var(0,0) + integrated_var(1,1) + integrated_var(2,2);
            double new_sum = covariance(0,0) + covariance(1,1) + covariance(2,2);
            // apply the correction factor on the largest contribution
            Eigen::Matrix3d corrected_covar;
            if (new_sum > integrated_sum)
                corrected_covar = covariance * alpha;
            else
            {
                corrected_covar = covariance;
                integrated_var *= alpha;
            }
            integrate_update_davis(position, integrated_pos, corrected_covar, integrated_var);

            ++continuous_samples;

            return ok;
        }

        const Eigen::Vector3d& pos() { return integrated_pos; }
        Eigen::Matrix3d var()        { return integrated_var; }

    private:
        float period;
        float last_sample_time;

        u32 continuous_samples;
        Eigen::Vector3d integrated_pos;
        Eigen::Matrix3d integrated_var;
    };

}

}