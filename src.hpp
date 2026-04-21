#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP

#include "math.h"
#include "monitor.h"

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    // Compute minimal squared distance between two disks centers within [0, TIME_INTERVAL]
    // assuming relative position p and relative velocity v.
    double min_dist_sqr_in_interval(const Vec &p, const Vec &v) const {
        // If relative speed is ~0, distance is constant
        double vnorm2 = v.x * v.x + v.y * v.y;
        if (vnorm2 <= 1e-12) return p.norm_sqr();
        // project time where distance minimal: t* = -p·v / |v|^2
        double t = -p.dot(v) / vnorm2;
        if (t < 0) t = 0;
        if (t > TIME_INTERVAL) t = TIME_INTERVAL;
        Vec d = p + v * t;
        return d.norm_sqr();
    }

    // Check if using candidate velocity will collide with robot j (using its last velocity)
    bool will_collide_with(int j, const Vec &v_cand) const {
        if (j == id) return false;
        Vec pj = monitor->get_pos_cur(j);
        Vec vj = monitor->get_v_cur(j);
        double rj = monitor->get_r(j);
        Vec dp = pos_cur - pj;
        Vec dv = v_cand - vj; // relative velocity assumption
        // If robots are separating (dp·dv >= 0) and not too close, likely safe
        // but we still compute min distance as in simulator to be consistent
        double min_s = min_dist_sqr_in_interval(dp, dv);
        double rr = r + rj - EPSILON; // keep a small margin
        return min_s <= rr * rr;
    }

    // Try to find a feasible velocity: desired first, then a couple sidesteps, else stop.
    Vec find_feasible_velocity(const Vec &v_des) const {
        // Priority rule: yield to lower-id robots if potential collision.
        auto safe_against = [&](const Vec &v_try, bool check_all) -> bool {
            int n = monitor->get_robot_number();
            for (int j = 0; j < n; ++j) {
                if (j == id) continue;
                if (!check_all && j >= id) continue; // yield only to lower ids
                if (will_collide_with(j, v_try)) return false;
            }
            return true;
        };

        // If too close to anyone (current), be conservative: stop.
        int n = monitor->get_robot_number();
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec dp = pos_cur - monitor->get_pos_cur(j);
            double rr = r + monitor->get_r(j);
            if (dp.norm_sqr() <= (rr - EPSILON) * (rr - EPSILON)) {
                return Vec();
            }
        }

        // 1) Try desired velocity
        if (safe_against(v_des, false)) return v_des;

        // 2) Try a couple of sidesteps perpendicular to the line to target
        Vec dir = v_des.norm() > 1e-9 ? v_des.normalize() : Vec(0, 0);
        Vec perp(dir.y, -dir.x);
        double side_speed = v_max * 0.6;
        Vec candidates[4] = {perp * side_speed, perp * (-side_speed), dir * (v_max * 0.4), Vec()};
        for (const auto &vc : candidates) {
            if (safe_against(vc, false)) return vc;
        }

        // 3) As a last resort, if blocked by lower-ids but free against all, try desired
        if (safe_against(v_des, true)) return v_des;

        // 4) Stop
        return Vec();
    }

public:

    Vec get_v_next() {
        // If already at target (within tolerance), stop
        Vec to_tar = pos_tar - pos_cur;
        double dist = to_tar.norm();
        if (dist <= EPSILON) {
            return Vec();
        }

        // Desired speed towards target, limited by v_max and distance over interval
        double max_step_speed = dist / TIME_INTERVAL;
        double speed = v_max;
        if (max_step_speed < speed) speed = max_step_speed;
        Vec v_des = to_tar.normalize() * speed;

        // Cap NaNs or infs safety
        if (!(v_des.x == v_des.x) || !(v_des.y == v_des.y)) {
            v_des = Vec();
        }

        // Try to find a feasible velocity avoiding lower-id robots
        Vec v_next = find_feasible_velocity(v_des);
        // Ensure speed cap (numerical)
        double vn = v_next.norm();
        if (vn > v_max) {
            v_next = v_next * (v_max / vn);
        }
        return v_next;
    }
};


#endif //PPCA_SRC_HPP

