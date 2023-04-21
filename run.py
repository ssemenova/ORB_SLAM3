import shlex, subprocess, time, os, signal

device_name = "laptop"
output_dir = '/home/sofiya/char/P3_dig_large'
datasets_dir = '/media/sofiya/Samsung_T5/'

prepend = "" # Optional prepend to results folder
systems = [
    #['orbslam3', '/home/sofiya/char/ORB_SLAM3'],
    #['kimera', '/home/sofiya/char/kimera_workspace'],
    ['openvins', '/home/sofiya/char/catkin_ws_ov']
]
datasets = [
        ['euroc', ['V2_02_medium']]
    #['euroc', ['MH_01_easy', 'MH_02_easy', 'MH_03_medium', 'MH_04_difficult', 'MH_05_difficult', 'V1_01_easy', 'V1_02_medium', 'V1_03_difficult', 'V2_01_easy', 'V2_02_medium', 'V2_03_difficult']]
    # ['hilti', ['exp06_construction_upper_level_3', 'exp16_attic_to_upper_gallery_2', 'exp10_cupola_2', 'exp18_corridor_lower_gallery_2', 'exp14_basement_2']]
]
frame_rates = [1]
multiple_repeat = 5

def run():
    for (dataset_name, all_seqs) in datasets:
        if "euroc" in dataset_name:
            base_frame_rate = 1
            reroute_topics = ""
            # ORBSLAM
            orbslam_type = "Stereo_Inertial"
            orbslam_calib_file_location = "Stereo-Inertial/EuRoC.yaml"
        elif "hilti" in dataset_name:
            base_frame_rate = 0.25
            reroute_topics = "/alphasense/cam0/image_raw:=/cam0/image_raw /alphasense/imu:=/imu0"
            # ORBSLAM
            orbslam_type = "Mono_Inertial"
            orbslam_calib_file_location = "Monocular-Inertial/hilti_mono"

        for seq_name in all_seqs:
            if "euroc" in dataset_name:
                short_seq_name = "".join(seq_name.split("_")[:2])
            elif "hilti" in dataset_name:
                short_seq_name = seq_name.split("_")[0]

            for (system_name, system_dir) in systems:
                for frame_rate in frame_rates:
                    if frame_rate == 1:
                        repeat = multiple_repeat
                    else:
                        repeat = 1

                    for i in range(0,repeat):
                        final_results_dir = os.path.join(
                            output_dir,
                            "{}{}_{}_{}_{}_{}".format(prepend, device_name, system_name, short_seq_name, frame_rate, i)
                        )

                        if system_name == "orbslam3":
                            parallel = "normal"
                            system_cmd = 'rosrun ORB_SLAM3 {} {}/Vocabulary/ORBvoc.txt {}/Examples/{} false'.format(orbslam_type, system_dir, system_dir, orbslam_calib_file_location)
                            save_cmd = 'mv KeyFrameTrajectory_TUM_Format.txt FrameTrajectory_KITTI_Format.txt FrameTrajectory_TUM_Format.txt output.txt {}'.format(final_results_dir)
                        elif system_name == "kimera":
                            parallel = "parallel"
                            system_cmd = 'roslaunch kimera_vio_ros kimera_vio_ros_{}.launch log_output_path:={}/results'.format(dataset_name, system_dir)
                            save_cmd = 'mv {}/results/output_frontend_stats.csv {}/results/traj_vio.csv {}/results/traj_pgo.csv output.txt {}'.format(system_dir, system_dir, system_dir, final_results_dir)
                        elif system_name == "openvins":
                            parallel = "4threads"
                            system_cmd = "roslaunch ov_msckf subscribe.launch config:={}_mav dosave:=true path_est:={}/results/trajectory.txt dotime:=true path_time:={}/results/time_provided.txt {}".format(dataset_name, system_dir, system_dir, slam_msckf_features_optional)
                            save_cmd = 'mv {}/results/trajectory.txt {}/results/time_provided.txt output.txt {}'.format(system_dir, system_dir, final_results_dir)

                        print("PLAYING... {} {} fps multiplier={}".format(system_name, short_seq_name, frame_rate))
                        # Run system... nonblocking
                        f = open("output.txt", "w")
                        system_proc = subprocess.Popen(shlex.split(system_cmd), stdout=f, stderr=f)
                        time.sleep(10) # Sleep to give system time to load vocabulary

                        # Run rosbag ... blocking
                        full_bag_path = os.path.join(datasets_dir, dataset_name, seq_name + ".bag")
                        bag_cmd = "rosbag play --rate {} {} {}".format(base_frame_rate * frame_rate, full_bag_path, reroute_topics)
                        print(bag_cmd)
                        subprocess.run(shlex.split(bag_cmd))
                        time.sleep(5)

                        # Kill system after rosbag is done
                        system_proc.send_signal(signal.SIGINT)
                        time.sleep(5) # Give some time to shut down
                        outs, errs = system_proc.communicate()

                        # Move results to the right directory
                        subprocess.run(shlex.split('mkdir {}'.format(final_results_dir))) # Make directory 
                        subprocess.run(shlex.split(save_cmd))

                        print("Saved ... ", save_cmd)
                        print("====================")




for max_slam in range(0, 101, 25):
    l = [int(max_slam/2), max_slam] if max_slam > 0 else [0]
    for max_slam_in_update in l:
        for max_msckf_in_update in range(0, 101, 25):
            if max_slam == 0 and max_msckf_in_update == 0:
                continue
            slam_msckf_features_optional = "max_slam:={} max_slam_in_update:={} max_msckf_in_update:={}".format(max_slam, max_slam_in_update, max_msckf_in_update)
            print(max_slam, max_slam_in_update, max_msckf_in_update)
            prepend = "{}_{}_{}_".format(max_slam, max_slam_in_update, max_msckf_in_update)

            final_results_dir = "{}{}_{}_{}_{}_{}".format(prepend, "laptop", "openvins", "V202", "1", "0")
            run()
