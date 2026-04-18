#include <xsf_math/xsf_math.hpp>
#include <cassert>
#include <cstdio>

using namespace xsf_math;

namespace {

void test_target_identity_binding() {
    printf("  target_identity_binding... ");

    track_manager tracker;
    tracker.params.mofn.m = 2;
    tracker.params.mofn.n = 2;

    detection det0;
    det0.position = {1000.0, 200.0, -50.0};
    det0.target_index = 7;

    const int track_id = tracker.start_tentative_track(det0, 0.0);
    const auto* started = tracker.find(track_id);
    assert(started != nullptr);
    assert(started->has_target_index());
    assert(started->target_index == 7);

    detection det1 = det0;
    det1.position.x += 20.0;

    auto update = tracker.update({det1}, 1.0);
    assert(update.confirmed_track_ids.size() == 1);
    assert(update.confirmed_track_ids.front() == track_id);
    assert(update.confirmed_tracks.size() == 1);
    assert(update.confirmed_tracks.front().track_id == track_id);
    assert(update.confirmed_tracks.front().has_target_index());
    assert(update.confirmed_tracks.front().target_index == 7);

    printf("OK\n");
}

void test_identity_refresh_from_associated_detection() {
    printf("  identity_refresh_from_associated_detection... ");

    track_manager tracker;
    tracker.params.mofn.m = 3;
    tracker.params.mofn.n = 3;

    detection det0;
    det0.position = {3000.0, -150.0, -400.0};

    const int track_id = tracker.start_tentative_track(det0, 0.0);
    const auto* started = tracker.find(track_id);
    assert(started != nullptr);
    assert(!started->has_target_index());

    detection det1 = det0;
    det1.position.x += 15.0;
    det1.target_index = 11;

    tracker.update({det1}, 1.0);

    const auto* updated = tracker.find(track_id);
    assert(updated != nullptr);
    assert(updated->has_target_index());
    assert(updated->target_index == 11);

    printf("OK\n");
}

}  // namespace

int main() {
    printf("=== Tracking Identity Tests ===\n");
    test_target_identity_binding();
    test_identity_refresh_from_associated_detection();
    printf("All tracking identity tests passed.\n");
    return 0;
}
