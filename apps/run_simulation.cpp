#include <iostream>
#include <fstream>
#include <string>

#include "tracker.hpp"
#include "highway_scenario.hpp"
#include "sensor_simulator.hpp"

int main(int argc, char** argv) {
    using namespace msf;

    int num_objects = 5;
    int num_steps = 300;
    double dt = 0.1;

    if (argc >= 2) {
        num_objects = std::stoi(argv[1]);
    }
    if (argc >= 3) {
        num_steps = std::stoi(argv[2]);
    }

    std::string out_dir = ".";
    if (argc >= 4) {
        out_dir = argv[3];
    }
    if (!out_dir.empty()) {
        char last = out_dir.back();
        if (last != '/' && last != '\\') {
            out_dir += "/";
        }
    }

    std::cout << "Running simulation: objects=" << num_objects
              << ", steps=" << num_steps << ", dt=" << dt << "s\n";
    std::cout << "Output directory: " << out_dir << "\n";

    HighwayScenario scenario(num_objects, dt);

    SensorSimulator sensor_sim(
        /*cam_std=*/1.0,
        /*radar_r_std=*/1.0,
        /*radar_angle_std=*/0.02,
        /*radar_vr_std=*/0.5,
        /*detection_prob=*/0.9,
        /*clutter_rate=*/0.1
    );

    TrackerParams params;
    params.process_noise_std = 1.0;
    params.cam_pos_noise_std = 1.0;
    params.radar_r_noise_std = 1.0;
    params.radar_angle_noise_std = 0.02;
    params.radar_vr_noise_std = 0.5;
    params.max_association_maha_dist = 16.0; // 게이트 조금 넉넉히
    params.max_missed = 5;
    params.min_hits_to_confirm = 3;

    MultiSensorTracker tracker(params);

    std::ofstream gt_file(out_dir + "ground_truth.csv");
    std::ofstream track_file(out_dir + "tracks.csv");
    std::ofstream det_file(out_dir + "detections.csv");

    if (!gt_file || !track_file || !det_file) {
        std::cerr << "Failed to open output CSV files. Check output directory." << std::endl;
        return 1;
    }

    gt_file << "time,obj_id,x,y,vx,vy\n";
    track_file << "time,track_id,x,y,vx,vy,confirmed,missed\n";
    det_file << "time,sensor,x,y,z2\n";

    for (int step = 0; step < num_steps; ++step) {
        scenario.step();
        double t = scenario.time();

        const auto& objs = scenario.objects();

        // ground truth 로그
        for (const auto& obj : objs) {
            gt_file << t << "," << obj.id << ","
                    << obj.x << "," << obj.y << ","
                    << obj.vx << "," << obj.vy << "\n";
        }

        auto detections = sensor_sim.generate(objs, t);

        // detection 로그
        for (const auto& d : detections) {
            det_file << t << ",";
            det_file << (d.sensor == SensorType::Camera ? "camera" : "radar") << ",";
            if (d.z.size() >= 2) {
                det_file << d.z(0) << "," << d.z(1) << ",";
            } else {
                det_file << "0,0,";
            }
            if (d.z.size() >= 3) {
                det_file << d.z(2);
            } else {
                det_file << "0";
            }
            det_file << "\n";
        }

        tracker.predict(t);
        tracker.update(detections);

        const auto& tracks = tracker.get_tracks();
        for (const auto& tr : tracks) {
            track_file << t << "," << tr.id << ","
                       << tr.x(0) << "," << tr.x(1) << ","
                       << tr.x(2) << "," << tr.x(3) << ","
                       << (tr.confirmed ? 1 : 0) << ","
                       << tr.missed << "\n";
        }
    }

    std::cout << "Simulation finished.\n";
    std::cout << "Generated files in: " << out_dir << "\n";

    return 0;
}
