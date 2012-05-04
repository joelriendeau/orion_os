#pragma once

#include <list>
#include <vector>

#include "Projections/projections.hpp"
#include "PatchedEigen/patched_eigen.hpp"

namespace stats {

namespace hidden_point {

    template <typename CovarBase = float>
    struct sample
    {
        Eigen::Vector3d coord; // ecef
        Eigen::Matrix<CovarBase, 3, 3> covar;
    };

    template <typename CovarBase = float>
    struct input
    {
        input() : horiz_variance(0), diagonal_distance(0) {}
        sample<CovarBase> sp;
        double horiz_distance; // horizontal distance from measured point to hidden point
        double horiz_variance; // as estimated by the surveyor, or sane default value for tape measures
                               // remember that for the normal distrib, 68% or the measurements fall within 1 standard deviation from the mean, and the variance is std_dev squared
        double diagonal_distance; // if not zero, will be used to solve the Z coordinate. not supported yet.
        double time_stamp;
    };

    namespace sol_warnings {
        static const u32 ok         = 0x00000000; // No warning
        static const u32 ambiguous  = 0x00000001; // Solution might be ambiguous
        static const u32 hunprecise = 0x00000002; // Poor horizontal precision 
        static const u32 vunprecise = 0x00000004; // Poor vertical precision
    }

    class solver
    {
    public:
        // find the best approximation for our hidden point using the covariance of the inputs as
        // a weighting guide
        template <class InputIteratorType, typename CovarBase>
        bool solve(const ellipsoids::ellipsoid& ell, const InputIteratorType inputs_begin, u32 inputs_size, sample<CovarBase>& solved, u32& warnings)
        {
            solved.coord.setZero();
            solved.covar.setZero();
            warnings = 0; // reset warnings

            if (inputs_size < 3) // iterative solver limited for 3 or more points. for two points, we need to know if the result is to the left or right.
            {
                warnings |= sol_warnings::ambiguous;
                return false;
            }

            // TODO : detect possibly very bad input data by comparing individual analytical solutions together -> rule out solving

            // transform input into local coordinates with first input as origin
            input<CovarBase> reference = *inputs_begin;
            Eigen::Vector3d ref_coord = reference.sp.coord;
            reference.sp.coord.setZero();

            // get the transform matrix to transform our coordinates into local space
            coordinates::earth_centered ecef;
            ecef.x = ref_coord(0);
            ecef.y = ref_coord(1);
            ecef.z = ref_coord(2);
            Eigen::Matrix3d rotation;
            coordinates::ecef_to_local_rot(ell, ecef, rotation);
            Eigen::Matrix3d rotation_transposed = rotation.transpose();

            // transform our first covariance matrix and insert into transformed vector
            // see http://www.ngs.noaa.gov/CORS/Articles/SolerChin1985.pdf
            reference.sp.covar = rotation * reference.sp.covar * rotation_transposed;
            std::vector< input<CovarBase> > transformed(inputs_size);
            transformed[0] = reference;

            // transform each other input in reference to the first input
            InputIteratorType it = inputs_begin;
            it++;
            u32 count = 1;
            for (;count < inputs_size; ++it, ++count)
            {
                input<CovarBase> inp = *it;
                inp.sp.coord -= ref_coord;
                inp.sp.coord = rotation * inp.sp.coord;
                inp.sp.covar = rotation * inp.sp.covar * rotation_transposed;
                transformed[count] = inp;
            }

            // find a good start approximation : compute both solutions for 2 points
            sample<CovarBase> o_right_a, o_left_a, o_right_b, o_left_b;
            u32 temp_warnings = 0;
            bool ok = analytical_solve_local(transformed[0], transformed[1], o_right_a, o_left_a, temp_warnings, false);
            if (!ok) return false;
            ok = analytical_solve_local(transformed[0], transformed[inputs_size-1], o_right_b, o_left_b, temp_warnings, false);
            if (!ok) return false;
            Eigen::Vector2d HP;
            if (!(temp_warnings & sol_warnings::hunprecise))
            {
                // both solutions are good:
                // solution is the one which comes back twice (or close)
                double drarb = (o_right_a.coord - o_right_b.coord).norm();
                double dralb = (o_right_a.coord - o_left_b.coord).norm();
                double dlarb = (o_left_a.coord - o_right_b.coord).norm();
                double dlalb = (o_left_a.coord - o_left_b.coord).norm();
                if ((drarb < dralb && drarb < dlarb && drarb < dlalb) || (dralb < dlarb && dralb < dlalb)) HP = o_right_a.coord.head<2>();
                else HP = o_left_a.coord.head<2>();
            }
            else
            {
                // warnings detected:
                // perform a ratio test in order to be sure
                // that there is no ambiguity in the solution
                double dab[4];
                dab[0] = (o_right_a.coord - o_right_b.coord).norm();
                dab[1] = (o_right_a.coord - o_left_b.coord).norm();
                dab[2] = (o_left_a.coord - o_right_b.coord).norm();
                dab[3] = (o_left_a.coord - o_left_b.coord).norm();
                u32 index[4] = {0, 1, 2, 3};
                // sort the values
                bool swap = true;
                while (swap)
                {
                    swap = false;
                    for (u32 i = 1; i < 4; i++)
                    {
                        if (dab[index[i]] < dab[index[i-1]])
                        {
                            u32 temp = index[i-1];
                            index[i-1] = index[i];
                            index[i] = temp;
                            swap = true;
                        }
                    }
                }
                if (4.*dab[index[0]] >= dab[index[1]])
                    warnings |= sol_warnings::ambiguous; // ratio test failed
                if (0 == index[0] || 1 == index[0]) HP = o_right_a.coord.head<2>();
                else HP = o_left_a.coord.head<2>();
            }

            // declare our work buffers
            Eigen::Vector2d dI;
            Eigen::MatrixXd H(inputs_size, 2); // observability matrix: from coords to projections towards the hidden point in this case.
            Eigen::MatrixXd HtRinv(2, inputs_size); // intermediary result
            Eigen::DiagonalMatrix<double, Eigen::Dynamic> R(inputs_size); // observation noise matrix
            Eigen::VectorXd Z(inputs_size); // current error matrix
            Eigen::MatrixXd J(2, inputs_size); // pseudo-inverse of observability matrix : the transition matrix
            Eigen::Vector2d dHP; // iteration
            double r;

            // iterate a maximum number of times
            for (u8 i = 0; i < 5; ++i)
            {
                for (u32 j = 0; j < inputs_size; ++j)
                {
                    dI = transformed[j].sp.coord.head<2>();
                    dI -= HP;
                    r = dI.norm();
                    dI /= r;
                    H.row(j) = dI.transpose();
                    Z(j) = r - transformed[j].horiz_distance;
                    // R(j, j) = dir.tranpose() * covar(x-y only) * dir
                    R.diagonal()(j) = (dI.transpose() * transformed[j].sp.covar.block<2,2>(0,0) * dI)(0) + transformed[j].horiz_variance;
                }
                HtRinv = H.transpose() * R.inverse();
                J = (HtRinv * H).inverse() * HtRinv;
                dHP = J * Z;
                HP = HP + dHP;

                if (dHP.norm() < 0.001)
                    break;
            }

            // verify horizontal precision
            Eigen::Matrix2d G;
            G = (H.transpose() * H).inverse();
            double PDOP2 = G(0,0) + G(1,1); // Squared position DOP
            if (PDOP2 >= 4.)
            {
                warnings |= sol_warnings::hunprecise;
            }
            else
            {
                // verify solution residuals
                for (u32 i = 0; i < inputs_size; ++i)
                {
                    if (Z(i) * Z(i) >= R.diagonal()(i) * 4.)
                    {
                        warnings |= sol_warnings::hunprecise;
                        break;
                    }
                }
            }

            // estimate vertical coordinate
            std::vector<double> z_coords(inputs_size);
            std::vector<double> z_vars(inputs_size);
            for (u32 i = 0; i < inputs_size; ++i)
            {
                z_coords[i] = transformed[i].sp.coord(2);
                z_vars[i] = transformed[i].sp.covar(2, 2);
            }
            double integration, var;
            stats::integration::integrate_scalar_davis(z_coords.begin(), z_coords.end(),
                                                       z_vars.begin(), z_vars.end(), integration, var);

            // verify vertical precision
            for (u32 i = 0; i < inputs_size; ++i)
            {
                double dz = transformed[i].sp.coord(2) - integration;
                if (dz * dz >= transformed[i].sp.covar(2, 2) * 4.)
                    warnings |= sol_warnings::vunprecise;
            }

            // transform back the new coordinate into ECEF
            solved.coord.head<2>() = HP;
            solved.coord(2) = integration;

            rotation = rotation.inverse().eval(); // eval called because of aliasing problem found at runtime
            solved.coord = rotation * solved.coord;
            solved.coord += ref_coord;

            // we finally need to compute the covariance of our new point :
            // we will transform our R error matrix using our transition matrix J
            Eigen::Matrix2d covar = J * R * J.transpose();

            solved.covar << covar, Eigen::Vector2d::Zero(),
                            Eigen::RowVector2d::Zero(), var;

            // transform the covariance matrix into ECEF as well
            solved.covar = rotation * solved.covar * rotation.transpose();

            return true;
        }

        template <typename CovarBase>
        static bool analytical_solve(const ellipsoids::ellipsoid& ell, const input<CovarBase>& iA, const input<CovarBase>& iB, sample<CovarBase>& o_right_of_ab, sample<CovarBase>& o_left_of_ab, u32& warnings)
        {
            input<double> transformedA, transformedB;
            warnings = 0; // reset warnings

            // this solution is always ambiguous
            warnings |= sol_warnings::ambiguous;

            // get the transform matrix to transform our coordinates into local space
            coordinates::earth_centered ecef;
            ecef.x = iA.sp.coord(0);
            ecef.y = iA.sp.coord(1);
            ecef.z = iA.sp.coord(2);
            Eigen::Matrix3d rotation;
            coordinates::ecef_to_local_rot(ell, ecef, rotation);
            Eigen::Matrix3d rotation_transposed = rotation.transpose();

            transformedA.horiz_distance = iA.horiz_distance;
            transformedA.horiz_variance = iA.horiz_variance;
            transformedA.diagonal_distance = iA.diagonal_distance;
            transformedA.sp.coord = Eigen::Vector3d::Zero();
            transformedA.sp.covar = rotation * iA.sp.covar * rotation_transposed;
            
            transformedB.horiz_distance = iB.horiz_distance;
            transformedB.horiz_variance = iB.horiz_variance;
            transformedB.diagonal_distance = iB.diagonal_distance;
            transformedB.sp.coord = rotation * (iB.sp.coord - iA.sp.coord);
            transformedB.sp.covar = rotation * iB.sp.covar * rotation_transposed;

            bool solved = analytical_solve_local(transformedA, transformedB, o_right_of_ab, o_left_of_ab, warnings, true);
            if (!solved) return false;

            Eigen::Matrix3d inv_rot = rotation.inverse();
            Eigen::Matrix3d inv_rot_transposed = inv_rot.transpose();
            o_right_of_ab.coord = (inv_rot * o_right_of_ab.coord) + iA.sp.coord;
            o_right_of_ab.covar = inv_rot * o_right_of_ab.covar * inv_rot_transposed;
            o_left_of_ab.coord = (inv_rot * o_left_of_ab.coord) + iA.sp.coord;
            o_left_of_ab.covar = inv_rot * o_left_of_ab.covar * inv_rot_transposed;

            return true;
        }

    private:
        template <typename CovarBase>
        static bool analytical_solve_local(const input<CovarBase>& iA, const input<CovarBase>& iB, sample<CovarBase>& o_right_of_ab, sample<CovarBase>& o_left_of_ab, u32& warnings, bool compute_covar = false)
        {
            u8 dim0 = 0, dim1 = 1;

            // this algorithm breaks if both points have same Y. so, change solving dimensions depending on the difference between same coordinates.
            Eigen::Vector3d AB;
            AB << iB.sp.coord.head<2>() - iA.sp.coord.head<2>(), 0;
            double diffx2 = AB(0) * AB(0);
            double diffy2 = AB(1) * AB(1);
            double distAB = std::sqrt(diffx2 + diffy2);
            
            if ((distAB + iA.horiz_distance <= iB.horiz_distance) ||
                (distAB + iB.horiz_distance <= iA.horiz_distance) ||
                (iA.horiz_distance + iB.horiz_distance <= distAB))
                return false; // inconsistent data, unsolvable

            if (diffx2 > diffy2)
            {
                dim0 = 1;
                dim1 = 0;
            }

            double Ax = iA.sp.coord(dim0);
            double Ay = iA.sp.coord(dim1);
            double Bx = iB.sp.coord(dim0);
            double By = iB.sp.coord(dim1);

            double Bx_m_Ax = Bx - Ax;
            double By_m_Ay = By - Ay;
            double Bx_m_Ax2 = Bx_m_Ax * Bx_m_Ax;
            double By_m_Ay2 = By_m_Ay * By_m_Ay;

            double dA2 = iA.horiz_distance * iA.horiz_distance;
            double dB2 = iB.horiz_distance * iB.horiz_distance;
            double Ax2 = Ax * Ax;
            double Ay2 = Ay * Ay;
            double Bx2 = Bx * Bx;
            double By2 = By * By;
            double Ka = (dA2 - dB2 + Bx2 + By2 - Ax2 - Ay2) / 2.;
            double Kb = dA2 - Ax2 - Ay2;

            double A = 1. + Bx_m_Ax2 / By_m_Ay2;
            double B = 2 * Ay * Bx_m_Ax / By_m_Ay - 2 * Ka * Bx_m_Ax / By_m_Ay2 - 2 * Ax;
            double C = Ka * Ka / By_m_Ay2 - 2 * Ay * Ka / By_m_Ay - Kb;

            double first_term = -B / (2 * A);
            double sqrt_term = std::sqrt(B * B - 4 * A * C) / (2 * A);

            double z_var;

            // find out which of the two solutions is the good one based on the location of 
            // the searched point being on the left or right of the AB segment when AB is
            // straight up, A at the bottom, B at the top
            o_right_of_ab.coord(dim0) = first_term - sqrt_term;
            o_right_of_ab.coord(dim1) = (Ka - o_right_of_ab.coord(dim0) * Bx_m_Ax) / By_m_Ay;
            if (!compute_covar)
            {
                o_right_of_ab.coord(2) = 0.5 * (iA.sp.coord(2) + iB.sp.coord(2));
            }
            else
            {
                // Variance weighted mean
                double _1_z_varA = 1. / iA.sp.covar(2, 2);
                double _1_z_varB = 1. / iB.sp.covar(2, 2);
                z_var = 1. / (_1_z_varA + _1_z_varB);
                o_right_of_ab.coord(2) = (iA.sp.coord(2) * _1_z_varA +
                                          iB.sp.coord(2) * _1_z_varB) * z_var;
                // Only take the best vertical variance
                if (iA.sp.covar(2, 2) <= iB.sp.covar(2, 2))
                    z_var = iA.sp.covar(2, 2);
                else
                    z_var = iB.sp.covar(2, 2);
            }
            Eigen::Vector3d AX;
            AX << o_right_of_ab.coord.head<2>() - iA.sp.coord.head<2>(), 0;
            Eigen::Vector3d cp = AB.cross(AX);
            if (cp(2) > 0)
            {
                o_left_of_ab.coord = o_right_of_ab.coord;
                o_right_of_ab.coord(dim0) = first_term + sqrt_term;
                o_right_of_ab.coord(dim1) = (Ka - o_right_of_ab.coord(dim0) * Bx_m_Ax) / By_m_Ay;
            }
            else
            {
                o_left_of_ab.coord(dim0) = first_term + sqrt_term;
                o_left_of_ab.coord(dim1) = (Ka - o_left_of_ab.coord(dim0) * Bx_m_Ax) / By_m_Ay;
                o_left_of_ab.coord(2) = o_right_of_ab.coord(2);
            }

            // verify horizontal precision
            Eigen::Matrix2d H;
            Eigen::Matrix2d G;
            H.row(0) = (iA.sp.coord.head<2>() - o_right_of_ab.coord.head<2>()).normalized();
            H.row(1) = (iB.sp.coord.head<2>() - o_right_of_ab.coord.head<2>()).normalized();
            G = (H.transpose() * H).inverse();
            double PDOP2 = G(0,0) + G(1,1); // squared Position DOP
            if (PDOP2 >= 4.) // corresponds to a 45° angle separation
                warnings |= sol_warnings::hunprecise;

            if (!compute_covar)
                return true;

            // compute the covariance for each solution
            Eigen::DiagonalMatrix<double, 2, 2> R;
            Eigen::Matrix2d J;

            R.diagonal()(0) = (H.row(0) * iA.sp.covar.block<2,2>(0,0) * H.row(0).transpose())(0) + iA.horiz_variance;
            R.diagonal()(1) = (H.row(0) * iB.sp.covar.block<2,2>(0,0) * H.row(0).transpose())(0) + iB.horiz_variance;
            J = G * H.transpose();
            o_right_of_ab.covar << J * R * J.transpose(), Eigen::Vector2d::Zero(), Eigen::RowVector2d::Zero(), z_var;

            // recheck horizontal precision (1/2)
            if (dA2 < R.diagonal()(0) * 4. || dB2 < R.diagonal()(1) * 4.)
                warnings |= sol_warnings::hunprecise;

            H.row(0) = (iA.sp.coord.head<2>() - o_left_of_ab.coord.head<2>()).normalized();
            H.row(1) = (iB.sp.coord.head<2>() - o_left_of_ab.coord.head<2>()).normalized();
            R.diagonal()(0) = (H.row(0) * iA.sp.covar.block<2,2>(0,0) * H.row(0).transpose())(0) + iA.horiz_variance;
            R.diagonal()(1) = (H.row(0) * iB.sp.covar.block<2,2>(0,0) * H.row(0).transpose())(0) + iB.horiz_variance;
            J = H.inverse();
            o_left_of_ab.covar << J * R * J.transpose(), Eigen::Vector2d::Zero(), Eigen::RowVector2d::Zero(), z_var;

            // recheck horizontal precision (2/2)
            if (dA2 < R.diagonal()(0) * 4. || dB2 < R.diagonal()(1) * 4.)
                warnings |= sol_warnings::hunprecise;

            // verify vertical precision
            double dz = o_right_of_ab.coord(2) - iA.sp.coord(2);
            if (dz * dz >= iA.sp.covar(2, 2) * 4.)
                warnings |= sol_warnings::vunprecise;
            dz = o_right_of_ab.coord(2) - iB.sp.coord(2);
            if (dz * dz >= iB.sp.covar(2, 2) * 4.)
                warnings |= sol_warnings::vunprecise;
            return true;
        }
    };
}

}