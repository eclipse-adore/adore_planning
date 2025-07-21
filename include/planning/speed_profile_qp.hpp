
#pragma once

#include <cassert>
#include <limits>
#include <map>
#include <vector>

#include <Eigen/Sparse>

#include "adore_map/route.hpp" // existing Route definition

#include "dynamics/traffic_participant.hpp" // existing participant set
#include <OsqpEigen/OsqpEigen.h>

namespace adore
{
namespace planner
{

class SpeedProfileQP
{
public:

  /* public interface identical to legacy one */
  void generate_from_route_and_participants( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                             double initial_speed, double initial_s, double initial_time, double max_lateral_acceleration,
                                             double desired_time_headway, double length /* horizon length [m] */ );

  /* query helpers – same as old version */
  double get_speed_at_s( double s ) const;
  double get_acc_at_s( double s ) const;

  adore::dynamics::Trajectory generate_trajectory( const map::Route& route, double time_step ) const;
  std::map<double, double>    s_to_speed;

private:

  /* internal helpers */
  struct EnvelopePoint
  {
    double s;
    double vmax;
  };

  std::vector<EnvelopePoint> sample_envelope( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                              double initial_s, double initial_time, double max_lat_acc, double desired_time_headway,
                                              double horizon_len ) const;

  void build_and_solve_qp( const std::vector<EnvelopePoint>& env, double v0, double amax, double amin );

  /* storage for profile – dense vectors and sparse map for random lookup */
  std::vector<double>      s_samples; // monotonically increasing s (m)
  std::vector<double>      v;         // solved speeds (m/s)
  std::vector<double>      a;         // central‑difference acceleration (m/s²)
  std::map<double, double> s_to_acc;
};

// ─────────────────────────────────────────────────────────────────────────────
//                                Implementation
// ─────────────────────────────────────────────────────────────────────────────
inline std::vector<SpeedProfileQP::EnvelopePoint>
SpeedProfileQP::sample_envelope( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants, double initial_s,
                                 double initial_time, double max_lat_acc, double desired_time_headway, double horizon_len ) const
{
  const double               base_max_speed = 13.0;
  constexpr double           k_ds           = 0.5; // 0.5 m resolution
  const int                  N              = static_cast<int>( horizon_len / k_ds ) + 1;
  std::vector<EnvelopePoint> env;
  env.reserve( N );
  double route_length = route.get_length();

  /* pre‑compute curvature / posting limits */
  for( int i = 0; i < N; ++i )
  {
    double s     = initial_s + i * k_ds;
    double vcurv = std::sqrt( max_lat_acc / std::max( std::fabs( route.get_curvature_at_s( s ) ), 1e-6 ) );

    auto   mp    = route.get_map_point_at_s( s );
    double vpost = mp.max_speed ? *mp.max_speed : base_max_speed;

    /* IDM gap: find nearest object in lane ahead at this s */
    double vidm = base_max_speed;
    // double ego_time_at_s = initial_time + ( i == 0 ? 0.0 : 1e9 ); // unknown yet – optimistic bound
    // for( const auto& [id, part] : traffic_participants.participants )
    // {
    //   const auto obj_state = part.trajectory ? part.trajectory->get_state_at_time( ego_time_at_s ) : part.state;
    //   double     obj_s     = route.get_s( obj_state );
    //   if( obj_s > s )
    //   {
    //     double gap     = obj_s - s;
    //     double desired = part.state.vx * desired_time_headway + 2.0; // simple headway
    //     double max_v   = part.state.vx * ( gap / desired );
    //     vidm           = std::min( vidm, std::max( 0.0, max_v ) );
    //   }
    // }

    double vlim = std::min( { vcurv, vpost, vidm } );
    if( s >= route_length )
      vlim = 0;

    env.push_back( { s, vlim } );
  }
  return env;
}

void
SpeedProfileQP::build_and_solve_qp( const std::vector<EnvelopePoint>& env, double v0, double amax, double amin )
{
  // ---------------------------------------------------------------------------
  // Build QP with BOTH hard accel/decel bounds (unchanged) and *soft comfort*
  // penalties that discourage running near those bounds.  We introduce two
  // nonnegative slack vectors s_plus (excess accel over comfort) and s_minus
  // (excess decel over comfort).  Hard bounds remain in the constraint set; the
  // slacks only appear in *soft* comfort rows and the objective.
  // ---------------------------------------------------------------------------
  const int N = static_cast<int>( env.size() );
  assert( N >= 2 );
  const int    Nseg = N - 1; // number of intervals
  const double k_ds = env[1].s - env[0].s;

  // ---- comfort envelope inside the hard limits --------------------------------
  // choose accel you are happy to use routinely (tune / expose params)
  const double a_soft_pos = 0.2 * amax;              // [m/s^2]
  const double a_soft_neg = 0.2 * std::fabs( amin ); // comfort braking magnitude (>0)

  // ---- objective weights ------------------------------------------------------
  const double w_prog     = 0.01; // progress reward (linear)
  const double w_slope    = 1.0;  // generic smoothing on dv
  const double w_acc_soft = 10.0; // quadratic penalty on s_plus
  const double w_dec_soft = 15.0; // quadratic penalty on s_minus (braking more costly)

  // total variables: speeds + accel slack + decel slack
  const int Nv     = N;
  const int Nvars  = Nv + 2 * Nseg;
  auto      idx_v  = [Nv]( int i ) { return i; };
  auto      idx_sp = [Nv]( int i ) { return Nv + i; };              // accel slack index
  auto      idx_sm = [Nv, Nseg]( int i ) { return Nv + Nseg + i; }; // decel slack index

  // ---------------- Hessian ----------------------------------------------------
  // We'll assemble triplets; all cross-terms zero except slope smoothing on v
  // and diagonal weights on the slack variables.
  std::vector<Eigen::Triplet<double>> H_trip;
  H_trip.reserve( 5 * N + 2 * Nseg );

  // slope smoothing: w_slope * (v_{i+1} - v_i)^2 expands to diag/off-diag
  for( int i = 0; i < Nseg; ++i )
  {
    double w = w_slope;
    H_trip.emplace_back( idx_v( i ), idx_v( i ), 2.0 * w );
    H_trip.emplace_back( idx_v( i ), idx_v( i + 1 ), -2.0 * w );
    H_trip.emplace_back( idx_v( i + 1 ), idx_v( i ), -2.0 * w );
    H_trip.emplace_back( idx_v( i + 1 ), idx_v( i + 1 ), 2.0 * w );
  }
  // slack diag penalties
  for( int i = 0; i < Nseg; ++i )
  {
    H_trip.emplace_back( idx_sp( i ), idx_sp( i ), w_acc_soft );
    H_trip.emplace_back( idx_sm( i ), idx_sm( i ), w_dec_soft );
  }

  Eigen::SparseMatrix<double> H( Nvars, Nvars );
  H.setFromTriplets( H_trip.begin(), H_trip.end() );

  // ---------------- Gradient ---------------------------------------------------
  // encourage higher speeds: f = -w_prog * 1 for v vars, 0 for slacks
  Eigen::VectorXd f = Eigen::VectorXd::Zero( Nvars );
  for( int i = 0; i < Nv; ++i )
    f[idx_v( i )] = -w_prog;

  // ---------------- Constraint matrix -----------------------------------------
  // Rows layout:
  //  0 .. Nv-1              envelope (v_i <= vmax_i)
  //  Nv .. Nv+Nseg-1        HARD accel (v_{i+1}-v_i <= amax*ds)
  //  ...                    HARD decel (v_i - v_{i+1} <= |amin|*ds)
  //  ...                    v >= 0
  //  ...                    v0 equality
  //  ...                    SOFT accel comfort (v_{i+1}-v_i - s_plus_i <= a_soft_pos*ds)
  //  ...                    SOFT decel comfort (v_i - v_{i+1} - s_minus_i <= a_soft_neg*ds)
  //  ...                    s_plus >= 0
  //  ...                    s_minus >= 0
  const int row_envelope = Nv;
  const int row_hacc     = row_envelope + Nseg;
  const int row_hdec     = row_hacc + Nseg;
  const int row_vnonneg  = row_hdec + Nseg;
  const int row_v0       = row_vnonneg + Nv;
  const int row_sacc     = row_v0 + 1;
  const int row_sdec     = row_sacc + Nseg;
  const int row_spnn     = row_sdec + Nseg;
  const int row_smnn     = row_spnn + Nseg;
  const int m            = row_smnn + Nseg; // total rows

  std::vector<Eigen::Triplet<double>> A_trip;
  A_trip.reserve( Nv + 4 * Nseg + Nv + 1 + 3 * Nseg );
  Eigen::VectorXd l = Eigen::VectorXd::Constant( m, -std::numeric_limits<double>::infinity() );
  Eigen::VectorXd u = Eigen::VectorXd::Constant( m, std::numeric_limits<double>::infinity() );
  int             row;

  // (a) envelope
  row = 0;
  for( int i = 0; i < Nv; ++i, ++row )
  {
    A_trip.emplace_back( row, idx_v( i ), 1.0 );
    double vmax = std::isfinite( env[i].vmax ) ? env[i].vmax : 1000.0;
    u[row]      = vmax;
  }
  // (b) HARD accel
  for( int i = 0; i < Nseg; ++i, ++row )
  {
    A_trip.emplace_back( row, idx_v( i + 1 ), 1.0 );
    A_trip.emplace_back( row, idx_v( i ), -1.0 );
    u[row] = amax * k_ds;
  }
  // (c) HARD decel
  for( int i = 0; i < Nseg; ++i, ++row )
  {
    A_trip.emplace_back( row, idx_v( i ), 1.0 );
    A_trip.emplace_back( row, idx_v( i + 1 ), -1.0 );
    u[row] = std::fabs( amin ) * k_ds;
  }
  // (d) v >= 0
  for( int i = 0; i < Nv; ++i, ++row )
  {
    A_trip.emplace_back( row, idx_v( i ), 1.0 );
    l[row] = 0.0; // u[row] = +inf already
  }
  // (e) v0 equality
  {
    A_trip.emplace_back( row, idx_v( 0 ), 1.0 );
    l[row] = u[row] = v0;
    ++row;
  }
  // (f) SOFT accel comfort (binds slack s_plus)
  for( int i = 0; i < Nseg; ++i, ++row )
  {
    A_trip.emplace_back( row, idx_v( i + 1 ), 1.0 );
    A_trip.emplace_back( row, idx_v( i ), -1.0 );
    A_trip.emplace_back( row, idx_sp( i ), -1.0 );
    u[row] = a_soft_pos * k_ds;
  }
  // (g) SOFT decel comfort (binds slack s_minus)
  for( int i = 0; i < Nseg; ++i, ++row )
  {
    A_trip.emplace_back( row, idx_v( i ), 1.0 );
    A_trip.emplace_back( row, idx_v( i + 1 ), -1.0 );
    A_trip.emplace_back( row, idx_sm( i ), -1.0 );
    u[row] = a_soft_neg * k_ds;
  }
  // (h) s_plus >= 0
  for( int i = 0; i < Nseg; ++i, ++row )
  {
    A_trip.emplace_back( row, idx_sp( i ), 1.0 );
    l[row] = 0.0;
  }
  // (i) s_minus >= 0
  for( int i = 0; i < Nseg; ++i, ++row )
  {
    A_trip.emplace_back( row, idx_sm( i ), 1.0 );
    l[row] = 0.0;
  }
  assert( row == m );

  Eigen::SparseMatrix<double> A( m, Nvars );
  A.setFromTriplets( A_trip.begin(), A_trip.end() );

  // ---------------- Solve ------------------------------------------------------
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity( false );
  solver.settings()->setWarmStart( true );
  solver.settings()->setMaxIteration( 80 );
  solver.settings()->setAbsoluteTolerance( 1e-4 );
  solver.settings()->setRelativeTolerance( 1e-4 );

  solver.data()->setNumberOfVariables( Nvars );
  solver.data()->setNumberOfConstraints( m );
  solver.data()->setHessianMatrix( H );
  solver.data()->setGradient( f );
  solver.data()->setLinearConstraintsMatrix( A );
  solver.data()->setLowerBound( l );
  solver.data()->setUpperBound( u );

  if( !solver.initSolver() )
    throw std::runtime_error( "OSQP init failed" );
  if( solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError )
    throw std::runtime_error( "OSQP solve failed" );

  const Eigen::VectorXd sol = solver.getSolution();

  // ---------------- Extract speed component back into profile ------------------
  s_samples.resize( N );
  v.resize( N );
  a.resize( N );
  s_to_speed.clear();
  s_to_acc.clear();
  for( int i = 0; i < N; ++i )
  {
    s_samples[i]             = env[i].s;
    v[i]                     = sol[idx_v( i )];
    a[i]                     = ( i == 0 ) ? 0.0 : ( v[i] - v[i - 1] ) / k_ds; // segment accel
    s_to_speed[s_samples[i]] = v[i];
    s_to_acc[s_samples[i]]   = a[i];
  }
}

inline void
SpeedProfileQP::generate_from_route_and_participants( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                                      double initial_speed, double initial_s, double initial_time,
                                                      double max_lateral_acceleration, double desired_time_headway, double length )
{
  const double amax = 2.0;  // TODO expose param
  const double amin = -3.0; // TODO expose param (negative)

  auto env = sample_envelope( route, traffic_participants, initial_s, initial_time, max_lateral_acceleration, desired_time_headway,
                              length );
  build_and_solve_qp( env, initial_speed, amax, amin );
}

inline double
SpeedProfileQP::get_speed_at_s( double s ) const
{
  auto it = s_to_speed.lower_bound( s );
  if( it == s_to_speed.begin() )
    return it->second;
  if( it == s_to_speed.end() )
    return std::prev( it )->second;
  auto   it_prev = std::prev( it );
  double frac    = ( s - it_prev->first ) / ( it->first - it_prev->first );
  return it_prev->second + frac * ( it->second - it_prev->second );
}

inline double
SpeedProfileQP::get_acc_at_s( double s ) const
{
  auto it = s_to_acc.lower_bound( s );
  if( it == s_to_acc.begin() )
    return it->second;
  if( it == s_to_acc.end() )
    return std::prev( it )->second;
  auto   it_prev = std::prev( it );
  double frac    = ( s - it_prev->first ) / ( it->first - it_prev->first );
  return it_prev->second + frac * ( it->second - it_prev->second );
}

inline adore::dynamics::Trajectory
SpeedProfileQP::generate_trajectory( const map::Route& route, double time_step ) const
{
  adore::dynamics::Trajectory coarse_traj;
  const std::size_t           N = s_samples.size();
  if( N < 2 )
    return coarse_traj;

  double accum_t = 0.0;
  for( std::size_t i = 0; i < N - 1; ++i )
  {
    double s1 = s_samples[i];
    double s2 = s_samples[i + 1];
    double v1 = v[i];
    double v2 = v[i + 1];
    double ds = s2 - s1;

    auto pose = route.get_pose_at_s( s1 );

    adore::dynamics::VehicleStateDynamic st;
    st.x         = pose.x;
    st.y         = pose.y;
    st.yaw_angle = pose.yaw;
    st.vx        = v1;
    st.ax        = a[i];
    st.time      = accum_t;
    coarse_traj.states.push_back( st );

    double vavg  = 0.5 * ( v1 + v2 );
    accum_t     += ( vavg > 1e-3 ) ? ds / vavg : 0.0;
  }
  // push final point
  {
    auto                                 pose_last = route.get_pose_at_s( s_samples.back() );
    adore::dynamics::VehicleStateDynamic st;
    st.x         = pose_last.x;
    st.y         = pose_last.y;
    st.yaw_angle = pose_last.yaw;
    st.vx        = v.back();
    st.ax        = a.back();
    st.time      = accum_t;
    coarse_traj.states.push_back( st );
  }

  // Resample to constant Δt grid using existing Trajectory interpolation
  adore::dynamics::Trajectory traj_out;
  for( double t = 0.0; t <= accum_t + 1e-6; t += time_step )
    traj_out.states.push_back( coarse_traj.get_state_at_time( t ) );


  return traj_out;
}
} // namespace planner
} // namespace adore
