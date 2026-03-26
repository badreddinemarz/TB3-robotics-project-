/**
 * robot_navigator.cpp
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  ROOT CAUSE FIX — why previous versions failed at obstacle 1:
 *  ─────────────────────────────────────────────────────────────
 *  Pure Pursuit computes:  w = 2 · v_sm · sin(α) / L
 *  At startup v_sm ≈ 0 (EMA-smoothed from 0), so w ≈ 0 regardless of α.
 *  The robot drove STRAIGHT east for 1–2 seconds before w built up — by
 *  then it was already hitting the crate at x=3.5.
 *  Additionally, a 1.60 m lookahead cuts corners at Zone 1, preventing
 *  the robot from ever reaching y=−0.82 as required.
 *
 *  THIS VERSION — algorithm:
 *  ─────────────────────────
 *  1. HEADING P-CONTROLLER (primary)
 *     w = Kp · α  — completely velocity-independent.
 *     At α=30°: w=0.94 rad/s from the very first tick.
 *     At α=65°: v=0 (rotate in place before moving forward).
 *     Speed:  v = V_MAX · max(cos(α),0) · obs_scale · goal_scale
 *
 *  2. PRECOMPUTED OPTIMAL WAYPOINTS
 *     Derived analytically from world.sdf obstacle geometry.
 *     Every clearance verified by hand (see table below).
 *     Tight zones use reduced speed and smaller capture radius.
 *
 *  3. VFH EMERGENCY DEFLECTION
 *     Fires only when an obstacle enters the 0.42 m forward bubble.
 *     Finds the nearest open sector to the goal direction and steers there.
 *     Hands back to the waypoint controller the moment the path is clear.
 *
 *  4. HARD EMERGENCY STOP
 *     If anything is within ESTOP_R of the robot: stop instantly, back up,
 *     rotate toward open space, resume.
 *
 *  5. STUCK RECOVERY
 *     If the robot has moved < 6 cm in 4 s: backup 0.6 s → rotate toward
 *     open space 1.2 s → rewind waypoint index → resume.
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  CLEARANCE TABLE (robot radius = 0.18 m)
 * ─────────────────────────────────────────────────────────────────────────
 *  Zone  Obstacle               Edge y      WP y     Clearance
 *  ─────────────────────────────────────────────────────────────────────
 *  Z1    crate S edge           −0.325      −0.88    0.555 m  ✓
 *  Z2    N-wall S face          +0.90       −0.50    1.40  m  ✓
 *  Z2    S-box  N edge          −1.75       −0.50    1.25  m  ✓
 *  Z3    N-pillar inner         +0.67       0.00     0.49  m  ✓
 *  Z3    S-pillar inner         −0.67       0.00     0.49  m  ✓
 *  Z4a   barrel_a S face        +0.51       −0.40    0.91  m  ✓
 *  Z4b   barrel_b N face        −0.51       +0.40    0.91  m  ✓
 *  Z5    keg S face             −0.30       −0.68    0.38  m  ✓
 *  Goal  post S at y=−3.5       −3.40       0.00     3.40  m  ✓
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <limits>

// ═══════════════════════════════════════════════════════════════════════════
//  MISSION
// ═══════════════════════════════════════════════════════════════════════════
static constexpr double GOAL_X   = 18.0;
static constexpr double GOAL_Y   =  0.0;
static constexpr double GOAL_TOL =  0.40;

// ═══════════════════════════════════════════════════════════════════════════
//  ROBOT GEOMETRY
// ═══════════════════════════════════════════════════════════════════════════
static constexpr double ROBOT_R   = 0.18;   // physical radius (m)
static constexpr double ESTOP_R   = 0.23;   // hard-stop bubble radius

// ═══════════════════════════════════════════════════════════════════════════
//  HEADING P-CONTROLLER
// ═══════════════════════════════════════════════════════════════════════════
static constexpr double K_HEAD    = 1.80;   // angular proportional gain (rad/s per rad)
static constexpr double W_MAX     = 2.00;   // maximum angular rate (rad/s)
// If heading error exceeds this, stop forward motion and rotate in place
static constexpr double ROTATE_THRESH = 65.0 * M_PI / 180.0;  // 65°

// ═══════════════════════════════════════════════════════════════════════════
//  VELOCITY
// ═══════════════════════════════════════════════════════════════════════════
static constexpr double V_MAX     = 0.28;   // normal top speed (m/s)
static constexpr double V_TIGHT   = 0.13;   // speed through tight/narrow zones
static constexpr double V_MIN     = 0.04;   // minimum forward speed when moving
static constexpr double OBS_BRAKE = 0.80;   // obs_scale = 1 when dFW >= this (m)
static constexpr double APPROACH_D= 0.80;   // start slowing at this dist to goal

// ═══════════════════════════════════════════════════════════════════════════
//  WAYPOINT TRACKING
// ═══════════════════════════════════════════════════════════════════════════
static constexpr double WP_R_LOOSE = 0.28;  // capture radius — normal waypoints
static constexpr double WP_R_TIGHT = 0.18;  // capture radius — tight waypoints

// ═══════════════════════════════════════════════════════════════════════════
//  VFH EMERGENCY DEFLECTION
// ═══════════════════════════════════════════════════════════════════════════
static constexpr double VFH_TRIGGER = 0.42;  // activate when dFW < this (m)
static constexpr double VFH_CLEAR   = 0.55;  // deactivate when dFW > this (m)

// ═══════════════════════════════════════════════════════════════════════════
//  STUCK / RECOVERY
// ═══════════════════════════════════════════════════════════════════════════
static constexpr double STUCK_T    = 4.0;   // declare stuck after this many seconds
static constexpr double STUCK_D    = 0.06;  // must move this far to reset timer
static constexpr double BACKUP_T   = 0.60;
static constexpr double ROTATE_T   = 1.20;


// ═══════════════════════════════════════════════════════════════════════════
//  WAYPOINTS  — analytically computed from world.sdf obstacle geometry
//
//  struct WP: {x, y, tight}
//    tight=true  → use V_TIGHT speed + WP_R_TIGHT capture radius
//    tight=false → use V_MAX   speed + WP_R_LOOSE capture radius
//
//  Robot starts at (0,0) facing east (yaw=0). Goal: (18,0).
// ═══════════════════════════════════════════════════════════════════════════
struct WP { double x, y; bool tight; };

// NOTE: The first waypoint (1.00, -0.25) makes the robot start turning
// SOUTH immediately on tick 1, BEFORE it ever reaches x=3.5.
// This is the critical fix — the heading error to this WP from (0,0) is
// atan2(-0.25, 1.00) = -14°. The P-controller immediately commands
// w = 1.8 * (-0.244) = -0.44 rad/s. The robot turns south from tick 1.

static const std::vector<WP> WPS = {
//   x       y      tight   comment
  {  1.00, -0.25,  false }, // turn south immediately — heading error = -14° at start
  {  2.20, -0.70,  false }, // well south, approaching Zone1
  {  3.50, -0.88,  true  }, // Z1: 0.555m south of crate edge (CRITICAL — must reach here)
  {  4.80, -0.70,  false }, // past crate, begin easing north
  {  5.50, -0.50,  false }, // pre-Zone2 approach corridor
  {  6.00, -0.50,  false }, // Z2: 1.40m south of N-wall, 1.25m north of S-box
  {  7.20, -0.22,  false }, // exit Zone2, drift back toward centreline
  {  8.30,  0.00,  false }, // approach Zone3 gate perfectly centred
  {  9.00,  0.00,  true  }, // Z3: 0.49m from each pillar (gate 0.98m inner width)
  { 10.00,  0.00,  false }, // exit gate cleanly
  { 11.00, -0.15,  false }, // pre-slalom: slight south setup
  { 12.00, -0.40,  true  }, // Z4a: 0.91m south of barrel_a inner surface
  { 12.60,  0.00,  false }, // transition: between the two barrels
  { 13.00,  0.40,  true  }, // Z4b: 0.91m north of barrel_b inner surface
  { 14.00,  0.00,  false }, // post-slalom: recentre
  { 14.80, -0.45,  false }, // pre-Zone5: approach keg from south side
  { 15.50, -0.68,  true  }, // Z5: 0.38m south of keg surface (keg r=0.30)
  { 16.50, -0.25,  false }, // past keg, ease back north
  { 17.50,  0.00,  false }, // final straight, locked on centreline
  { 18.00,  0.00,  false }, // GOAL
};


// ═══════════════════════════════════════════════════════════════════════════
class RaceNavigator : public rclcpp::Node
{
public:
    RaceNavigator() : Node("race_navigator")
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&RaceNavigator::odomCb, this, std::placeholders::_1));
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&RaceNavigator::scanCb, this, std::placeholders::_1));
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),   // 20 Hz
            std::bind(&RaceNavigator::loop, this));

        RCLCPP_INFO(get_logger(),
            "[RaceNav] READY — %zu waypoints — goal=(%.1f,%.1f) tol=%.2fm",
            WPS.size(), GOAL_X, GOAL_Y, GOAL_TOL);
        RCLCPP_INFO(get_logger(),
            "[RaceNav] Controller: heading-P (Kp=%.2f) — no Pure Pursuit", K_HEAD);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr                                  timer_;

    // ── Pose ──────────────────────────────────────────────────────────────
    double x_ = 0, y_ = 0, yaw_ = 0;
    bool   got_odom_ = false, got_scan_ = false;

    // ── LiDAR sectors ─────────────────────────────────────────────────────
    //  dF_  : min range in ±20°   (tight forward cone)
    //  dFW_ : min range in ±40°   (wide forward — used for braking/VFH)
    //  dL_, dR_: min range in 0..100° and -100..0° (for recovery direction)
    float  rmin_ = 0.05f;
    double dF_   = 9.9;
    double dFW_  = 9.9;
    double dL_   = 9.9;
    double dR_   = 9.9;

    // World-frame hit points (for VFH sector scan)
    struct Pt { double wx, wy; };
    std::vector<Pt> hits_;

    // ── Navigation state ──────────────────────────────────────────────────
    int  wp_idx_ = 0;
    bool done_   = false;

    // ── State machine ─────────────────────────────────────────────────────
    enum class Phase {
        NAVIGATE,         // normal: heading P-controller tracking waypoints
        VFH_DEFLECT,      // unexpected obstacle close ahead: steer to open sector
        ESTOP,            // too close: stop, back up, turn to open space
        RECOVER_BACKUP,   // stuck recovery: reverse
        RECOVER_ROTATE,   // stuck recovery: rotate toward open sector
        DONE
    };
    Phase phase_ = Phase::NAVIGATE;

    // ── Stuck detection ───────────────────────────────────────────────────
    double       sk_rx_ = 0, sk_ry_ = 0;
    rclcpp::Time sk_t_;
    bool         sk_init_  = false;
    int          rec_sign_ = 1;
    rclcpp::Time rec_t_;


    // ═════════════════════════════════════════════════════════════════════
    //  UTILITIES
    // ═════════════════════════════════════════════════════════════════════

    // Normalize angle to (−π, +π]
    static double na(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void pub(double v, double w) {
        geometry_msgs::msg::Twist t;
        t.linear.x  = std::clamp(v, -0.15, V_MAX);
        t.angular.z = std::clamp(w, -W_MAX, W_MAX);
        cmd_pub_->publish(t);
    }
    void stop() { pub(0.0, 0.0); }

    // ═════════════════════════════════════════════════════════════════════
    //  ODOMETRY
    // ═════════════════════════════════════════════════════════════════════
    void odomCb(const nav_msgs::msg::Odometry::SharedPtr m)
    {
        x_ = m->pose.pose.position.x;
        y_ = m->pose.pose.position.y;
        tf2::Quaternion q(
            m->pose.pose.orientation.x, m->pose.pose.orientation.y,
            m->pose.pose.orientation.z, m->pose.pose.orientation.w);
        double r, p;
        tf2::Matrix3x3(q).getRPY(r, p, yaw_);
        got_odom_ = true;
    }

    // ═════════════════════════════════════════════════════════════════════
    //  SCAN
    // ═════════════════════════════════════════════════════════════════════
    void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr m)
    {
        if (!got_odom_) return;
        rmin_ = m->range_min;
        float rmax = std::min(m->range_max, 5.5f);

        dF_ = dFW_ = dL_ = dR_ = 9.9;
        hits_.clear();
        hits_.reserve(m->ranges.size());

        int n = (int)m->ranges.size();
        for (int i = 0; i < n; i++) {
            float r = m->ranges[i];
            if (!std::isfinite(r) || r < rmin_ || r > rmax || r <= 0.f) continue;

            double ang = m->angle_min + i * m->angle_increment;  // robot frame
            hits_.push_back({x_ + r * std::cos(yaw_ + ang),
                             y_ + r * std::sin(yaw_ + ang)});

            double a = std::fabs(ang);
            if (a <  20.0 * M_PI/180.0)               dF_  = std::min(dF_,  (double)r);
            if (a <  40.0 * M_PI/180.0)               dFW_ = std::min(dFW_, (double)r);
            if (ang > 0 && a < 100.0 * M_PI/180.0)   dL_  = std::min(dL_,  (double)r);
            if (ang < 0 && a < 100.0 * M_PI/180.0)   dR_  = std::min(dR_,  (double)r);
        }
        got_scan_ = true;
    }

    // ═════════════════════════════════════════════════════════════════════
    //  HEADING P-CONTROLLER — steers toward (tx, ty)
    //
    //  Returns {v, w}.
    //  If heading error > ROTATE_THRESH: v=0, full turn toward target.
    //  Otherwise: v proportional to cos(alpha), w proportional to alpha.
    //  Speed further reduced by obstacle proximity and goal proximity.
    // ═════════════════════════════════════════════════════════════════════
    struct Cmd { double v, w; };

    Cmd headingControl(double tx, double ty, bool tight)
    {
        double desired = std::atan2(ty - y_, tx - x_);
        double alpha   = na(desired - yaw_);   // heading error

        // Angular: simple P-controller, INDEPENDENT of velocity
        double w = std::clamp(K_HEAD * alpha, -W_MAX, W_MAX);

        // If too far off heading → rotate in place (v=0)
        if (std::fabs(alpha) > ROTATE_THRESH) {
            return {0.0, w};
        }

        // Linear: scale by alignment, obstacle proximity, goal proximity
        double v_base   = tight ? V_TIGHT : V_MAX;
        double align    = std::max(std::cos(alpha), 0.0);              // 1 ahead, 0 at 90°
        double obs_sc   = std::min(dFW_ / OBS_BRAKE, 1.0);            // slow near obstacles
        double goal_d   = std::hypot(GOAL_X - x_, GOAL_Y - y_);
        double goal_sc  = (goal_d < APPROACH_D)
                          ? std::max(goal_d / APPROACH_D, 0.35) : 1.0;

        double v = std::max(v_base * align * obs_sc * goal_sc, V_MIN);

        // Extra safety: if something very close directly ahead, cut speed hard
        if (dF_ < ESTOP_R + 0.08) v = 0.0;

        return {v, w};
    }

    // ═════════════════════════════════════════════════════════════════════
    //  VFH — find the nearest open sector to the goal direction
    //
    //  Scans the hit-point cloud in 15° bins (24 bins × 360°).
    //  Returns the world-frame angle of the most open sector nearest
    //  the goal direction.
    // ═════════════════════════════════════════════════════════════════════
    double vfhOpenAngle(double prefer_dir) const
    {
        static constexpr int    BINS   = 24;
        static constexpr double BIN_W  = 2.0 * M_PI / BINS;  // 15°

        // Compute minimum range in each bin
        std::array<double, BINS> bin_min;
        bin_min.fill(9.9);

        for (auto &h : hits_) {
            double world_ang = std::atan2(h.wy - y_, h.wx - x_);
            double norm_ang  = world_ang;
            if (norm_ang < 0) norm_ang += 2.0 * M_PI;
            int b = (int)(norm_ang / BIN_W) % BINS;
            double d = std::hypot(h.wx - x_, h.wy - y_);
            bin_min[b] = std::min(bin_min[b], d);
        }

        // Find the open bin nearest to prefer_dir
        double best_ang  = prefer_dir;
        double best_cost = 1e9;

        for (int b = 0; b < BINS; b++) {
            if (bin_min[b] < VFH_CLEAR) continue;   // blocked

            double bangle     = b * BIN_W;   // 0..2π
            double world_bangle = na(bangle);
            double angular_dist = std::fabs(na(world_bangle - prefer_dir));
            double cost         = angular_dist;   // prefer nearest to goal direction

            if (cost < best_cost) {
                best_cost = cost;
                best_ang  = world_bangle;
            }
        }
        return best_ang;
    }

    // ═════════════════════════════════════════════════════════════════════
    //  OPEN SECTOR — widest open direction (for escape/recovery)
    // ═════════════════════════════════════════════════════════════════════
    double openSector() const
    {
        static constexpr int    BINS  = 24;
        static constexpr double BIN_W = 2.0 * M_PI / BINS;

        std::array<double, BINS> bin_min;
        bin_min.fill(9.9);

        for (auto &h : hits_) {
            double a = std::atan2(h.wy - y_, h.wx - x_);
            if (a < 0) a += 2.0 * M_PI;
            int b = (int)(a / BIN_W) % BINS;
            bin_min[b] = std::min(bin_min[b], std::hypot(h.wx-x_, h.wy-y_));
        }

        int   best_b   = 0;
        double best_cl = 0.0;
        for (int b = 0; b < BINS; b++) {
            if (bin_min[b] > best_cl) { best_cl = bin_min[b]; best_b = b; }
        }
        return na(best_b * BIN_W);
    }

    // ═════════════════════════════════════════════════════════════════════
    //  STUCK DETECTION
    // ═════════════════════════════════════════════════════════════════════
    bool isStuck()
    {
        if (!sk_init_) {
            sk_rx_ = x_; sk_ry_ = y_; sk_t_ = now();
            sk_init_ = true; return false;
        }
        if (std::hypot(x_ - sk_rx_, y_ - sk_ry_) > STUCK_D) {
            sk_rx_ = x_; sk_ry_ = y_; sk_t_ = now(); return false;
        }
        return (now() - sk_t_).seconds() > STUCK_T;
    }

    // Rewind wp_idx to closest waypoint that is still ahead of robot
    void rewindWaypoint()
    {
        // First find the closest waypoint overall
        int   closest    = wp_idx_;
        double closest_d = 1e9;
        for (int i = std::max(0, wp_idx_ - 3); i < (int)WPS.size(); i++) {
            double d = std::hypot(WPS[i].x - x_, WPS[i].y - y_);
            if (d < closest_d) { closest_d = d; closest = i; }
        }
        // Ensure it's ahead of us (positive dot product with our heading)
        while (closest > 0) {
            double dx  = WPS[closest].x - x_, dy = WPS[closest].y - y_;
            double dot = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
            if (dot > 0.0) break;
            closest--;
        }
        wp_idx_ = closest;
        RCLCPP_INFO(get_logger(), "[RaceNav] Rewind → wp_idx=%d", wp_idx_);
    }

    // ═════════════════════════════════════════════════════════════════════
    //  WAYPOINT ADVANCE
    //  Called every tick. Advances wp_idx when robot is close enough to
    //  current waypoint OR when waypoint has been passed (dot < 0).
    // ═════════════════════════════════════════════════════════════════════
    void advanceWaypoint()
    {
        while (wp_idx_ < (int)WPS.size()) {
            double dx   = WPS[wp_idx_].x - x_;
            double dy   = WPS[wp_idx_].y - y_;
            double dist = std::hypot(dx, dy);
            double dot  = std::cos(yaw_) * dx + std::sin(yaw_) * dy;

            double capture_r = WPS[wp_idx_].tight ? WP_R_TIGHT : WP_R_LOOSE;

            // Advance if: within capture radius, OR waypoint is now behind us
            if (dist < capture_r || (dist < capture_r * 2.5 && dot < 0.0)) {
                RCLCPP_INFO(get_logger(),
                    "[RaceNav] WP %d captured (%.2f,%.2f)  dist=%.2f  dot=%.2f",
                    wp_idx_, WPS[wp_idx_].x, WPS[wp_idx_].y, dist, dot);
                wp_idx_++;
            } else {
                break;
            }
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    //  MAIN LOOP — 20 Hz
    // ═════════════════════════════════════════════════════════════════════
    void loop()
    {
        if (!got_odom_ || !got_scan_) return;
        if (done_) { stop(); return; }

        const double goal_d = std::hypot(GOAL_X - x_, GOAL_Y - y_);

        // ── Goal reached? ───────────────────────────────────────────────
        if (goal_d < GOAL_TOL) { finish(goal_d); return; }

        switch (phase_) {

        // ╔═════════════════════════════════════════════════════════════╗
        // ║  NAVIGATE — heading P-controller following waypoints        ║
        // ╚═════════════════════════════════════════════════════════════╝
        case Phase::NAVIGATE:
        {
            // ── Hard emergency stop
            if (dF_ < ESTOP_R || dFW_ < ESTOP_R - 0.02) {
                RCLCPP_WARN(get_logger(),
                    "[RaceNav] ESTOP  dF=%.3f  dFW=%.3f", dF_, dFW_);
                stop();
                phase_ = Phase::ESTOP;
                break;
            }

            // ── Stuck?
            if (isStuck()) {
                RCLCPP_WARN(get_logger(), "[RaceNav] STUCK → RECOVER");
                rec_t_    = now();
                rec_sign_ = (dL_ > dR_) ? 1 : -1;
                sk_init_  = false;
                phase_    = Phase::RECOVER_BACKUP;
                break;
            }

            // ── VFH deflection: unexpected close obstacle ahead
            if (dFW_ < VFH_TRIGGER) {
                RCLCPP_WARN(get_logger(),
                    "[RaceNav] VFH deflect  dFW=%.3f", dFW_);
                phase_ = Phase::VFH_DEFLECT;
                break;
            }

            // ── Advance waypoint index
            advanceWaypoint();

            if (wp_idx_ >= (int)WPS.size()) {
                // All waypoints consumed — steer straight to goal
                Cmd c = headingControl(GOAL_X, GOAL_Y, false);
                pub(c.v, c.w);
                break;
            }

            // ── Compute command toward current waypoint
            const WP &wp = WPS[wp_idx_];
            Cmd c = headingControl(wp.x, wp.y, wp.tight);
            pub(c.v, c.w);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "[NAV] pos=(%.2f,%.2f) yaw=%.0f°  wp=%d(%.2f,%.2f)  "
                "goal=%.2fm  v=%.2f  w=%.2f  dF=%.2f  dFW=%.2f",
                x_, y_, yaw_*180/M_PI,
                wp_idx_, wp.x, wp.y, goal_d,
                c.v, c.w, dF_, dFW_);
            break;
        }

        // ╔═════════════════════════════════════════════════════════════╗
        // ║  VFH_DEFLECT — unexpected obstacle, steer around it        ║
        // ╚═════════════════════════════════════════════════════════════╝
        case Phase::VFH_DEFLECT:
        {
            // Check emergency escalation
            if (dF_ < ESTOP_R) {
                stop(); phase_ = Phase::ESTOP; break;
            }

            // Return to normal navigation when path is clear
            if (dFW_ > VFH_CLEAR) {
                RCLCPP_INFO(get_logger(), "[RaceNav] VFH cleared → NAVIGATE");
                advanceWaypoint();   // skip any waypoints passed during deflection
                sk_init_ = false;
                phase_   = Phase::NAVIGATE;
                break;
            }

            // Find open sector nearest to goal direction
            double goal_dir = std::atan2(GOAL_Y - y_, GOAL_X - x_);
            double open_dir = vfhOpenAngle(goal_dir);

            // Steer toward open sector at reduced speed
            Cmd c = headingControl(
                x_ + 2.0 * std::cos(open_dir),
                y_ + 2.0 * std::sin(open_dir),
                true);   // tight=true → slow speed
            pub(c.v, c.w);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
                "[VFH] open_dir=%.0f°  dFW=%.2f  v=%.2f  w=%.2f",
                open_dir*180/M_PI, dFW_, c.v, c.w);
            break;
        }

        // ╔═════════════════════════════════════════════════════════════╗
        // ║  ESTOP — something too close, back up toward open space    ║
        // ╚═════════════════════════════════════════════════════════════╝
        case Phase::ESTOP:
        {
            if (dF_ > ESTOP_R + 0.12 && dFW_ > ESTOP_R + 0.10) {
                RCLCPP_INFO(get_logger(), "[RaceNav] ESTOP cleared → NAVIGATE");
                sk_init_ = false;
                phase_   = Phase::NAVIGATE;
                break;
            }
            // Back up slowly toward open space
            double open   = openSector();
            double behind = na(open - M_PI);   // opposite of open = behind us
            double turn   = na(behind - yaw_);
            // Slow reverse + gentle rotation to face away from obstacle
            pub(-0.08, std::clamp(turn * 1.2, -W_MAX, W_MAX));
            break;
        }

        // ╔═════════════════════════════════════════════════════════════╗
        // ║  RECOVER_BACKUP — reverse straight                         ║
        // ╚═════════════════════════════════════════════════════════════╝
        case Phase::RECOVER_BACKUP:
        {
            if ((now() - rec_t_).seconds() < BACKUP_T) {
                pub(-0.10, 0.0);
            } else {
                rec_t_ = now();
                phase_ = Phase::RECOVER_ROTATE;
            }
            break;
        }

        // ╔═════════════════════════════════════════════════════════════╗
        // ║  RECOVER_ROTATE — turn toward open space, then resume      ║
        // ╚═════════════════════════════════════════════════════════════╝
        case Phase::RECOVER_ROTATE:
        {
            if ((now() - rec_t_).seconds() < ROTATE_T) {
                double open = openSector();
                double err  = na(open - yaw_);
                // If we found a good direction, turn toward it; else spin
                double w = (std::fabs(err) > 0.15)
                           ? std::clamp(err * 2.0, -W_MAX, W_MAX)
                           : rec_sign_ * 1.2;
                pub(0.04, w);
            } else {
                rewindWaypoint();
                sk_init_ = false;
                phase_   = Phase::NAVIGATE;
                RCLCPP_INFO(get_logger(),
                    "[RaceNav] Recovery complete → NAVIGATE  wp=%d", wp_idx_);
            }
            break;
        }

        case Phase::DONE:
            stop();
            break;
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    //  FINISH
    // ═════════════════════════════════════════════════════════════════════
    void finish(double err)
    {
        stop();
        done_  = true;
        phase_ = Phase::DONE;

        RCLCPP_INFO(get_logger(), " ");
        RCLCPP_INFO(get_logger(),
            "╔══════════════════════════════════════════════════╗");
        RCLCPP_INFO(get_logger(),
            "║   ██████  FINISH LINE REACHED  ██████           ║");
        RCLCPP_INFO(get_logger(),
            "║   position = (%.3f, %.3f)                       ║", x_, y_);
        RCLCPP_INFO(get_logger(),
            "║   goal_err = %.3f m   wps_done = %d/%zu         ║",
            err, wp_idx_, WPS.size());
        RCLCPP_INFO(get_logger(),
            "╚══════════════════════════════════════════════════╝");
    }
};


// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RaceNavigator>());
    rclcpp::shutdown();
    return 0;
}
