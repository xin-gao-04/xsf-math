#include <xsf_math/xsf_math.hpp>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace xsf_math;

static bool approx(double a, double b, double tol = 1e-6) {
    return std::abs(a - b) < tol;
}

static bool approx_rel(double a, double b, double tol = 0.01) {
    if (std::abs(b) < 1e-20) return std::abs(a) < tol;
    return std::abs((a - b) / b) < tol;
}

void test_vec3() {
    printf("  vec3... ");
    vec3 a{1, 2, 3}, b{4, 5, 6};
    auto c = a + b;
    assert(approx(c.x, 5) && approx(c.y, 7) && approx(c.z, 9));

    assert(approx(a.dot(b), 32.0));

    auto cross = a.cross(b);
    assert(approx(cross.x, -3) && approx(cross.y, 6) && approx(cross.z, -3));

    assert(approx(vec3{3,4,0}.magnitude(), 5.0));

    auto n = vec3{0, 0, 5}.normalized();
    assert(approx(n.z, 1.0));
    printf("OK\n");
}

void test_mat3() {
    printf("  mat3... ");
    auto I = mat3::identity();
    vec3 v{1, 2, 3};
    auto r = I * v;
    assert(approx(r.x, 1) && approx(r.y, 2) && approx(r.z, 3));

    auto M = mat3::identity();
    auto Mt = M.transposed();
    assert(approx(Mt.determinant(), 1.0));
    printf("OK\n");
}

void test_coordinate_transform() {
    printf("  coordinate_transform... ");
    euler_angles zero = {0, 0, 0};
    auto dcm = dcm_wcs_to_ecs(zero);
    // With zero angles, WCS=ECS (identity)
    assert(approx(dcm.m[0][0], 1.0));
    assert(approx(dcm.m[1][1], 1.0));
    assert(approx(dcm.m[2][2], 1.0));

    // 90 deg heading (facing East): North in WCS maps to -Y (left) in ECS
    euler_angles h90 = {constants::half_pi, 0, 0};
    vec3 north{1, 0, 0};
    auto ecs = wcs_to_ecs(north, h90);
    assert(approx(ecs.y, -1.0, 1e-5));  // North -> left when heading East

    // Azimuth/elevation
    vec3 ne{1, 1, 0};
    double az = azimuth_from_vec(ne);
    assert(approx(az, constants::pi / 4.0, 1e-5));  // 45 degrees

    // Great circle distance
    lla a = {0, 0, 0}, b = {0, constants::pi / 180.0, 0};  // 1 degree longitude at equator
    double d = great_circle_distance(a, b);
    assert(approx_rel(d, 111195.0, 0.01));  // ~111 km

    printf("OK\n");
}

void test_atmosphere() {
    printf("  atmosphere... ");
    // Sea level
    assert(approx_rel(atmosphere::temperature(0), 288.15, 0.001));
    assert(approx_rel(atmosphere::pressure(0), 101325.0, 0.001));
    assert(approx_rel(atmosphere::density(0), 1.225, 0.01));
    assert(approx_rel(atmosphere::sonic_velocity(0), 340.3, 0.01));

    // Tropopause
    assert(approx_rel(atmosphere::temperature(11000), 216.65, 0.001));

    // Density decreases with altitude
    assert(atmosphere::density(5000) < atmosphere::density(0));
    assert(atmosphere::density(10000) < atmosphere::density(5000));

    // Mach number
    double mach = atmosphere::mach_number(0, 340.3);
    assert(approx_rel(mach, 1.0, 0.01));

    printf("OK\n");
}

void test_interpolation() {
    printf("  interpolation... ");
    assert(approx(lerp(0, 10, 0.5), 5.0));
    assert(approx(clamp(5.0, 0.0, 3.0), 3.0));

    std::vector<double> xs = {0, 1, 2, 3};
    std::vector<double> ys = {0, 10, 20, 30};
    assert(approx(table_lookup(xs, ys, 1.5), 15.0));
    assert(approx(table_lookup(xs, ys, 0.0), 0.0));
    assert(approx(table_lookup(xs, ys, 3.0), 30.0));

    printf("OK\n");
}

void test_db_conversions() {
    printf("  dB conversions... ");
    assert(approx_rel(db_to_linear(10.0), 10.0, 0.001));
    assert(approx_rel(db_to_linear(20.0), 100.0, 0.001));
    assert(approx_rel(db_to_linear(30.0), 1000.0, 0.001));
    assert(approx_rel(linear_to_db(100.0), 20.0, 0.001));
    printf("OK\n");
}

void test_kalman_filter() {
    printf("  kalman_filter... ");
    kalman_filter_6state kf;
    kf.meas_noise[0] = kf.meas_noise[1] = kf.meas_noise[2] = 10.0;
    kf.process_noise[0] = kf.process_noise[1] = kf.process_noise[2] = 0.1;

    // Target moving at constant velocity
    vec3 true_pos = {0, 0, 0};
    vec3 true_vel = {100, 50, -10};

    for (int i = 0; i < 50; ++i) {
        double t = i * 0.1;
        vec3 meas = true_pos + true_vel * t;
        // Add some noise (deterministic for test)
        meas.x += (i % 5 - 2) * 2.0;
        meas.y += (i % 3 - 1) * 3.0;
        kf.update(t, meas);
    }

    // After 50 updates, velocity estimate should be close
    vec3 est_vel = kf.velocity();
    assert(approx_rel(est_vel.x, 100.0, 0.1));
    assert(approx_rel(est_vel.y, 50.0, 0.1));

    printf("OK\n");
}

int main() {
    printf("=== Core Module Tests ===\n");
    test_vec3();
    test_mat3();
    test_coordinate_transform();
    test_atmosphere();
    test_interpolation();
    test_db_conversions();
    test_kalman_filter();
    printf("All core tests passed.\n");
    return 0;
}
