import shlex, subprocess, time, os, signal

device_name = "laptop"
output_dir = '/home/sofiya/char/orbslam_types'
datasets_dir = '/media/sofiya/Samsung_T5/'

prepend = "" # Optional prepend to results folder
systems = [
    ['orbslam3', '/home/sofiya/char/ORB_SLAM3'],
    #['kimera', '/home/sofiya/char/kimera_workspace'],
    #['openvins', '/home/sofiya/char/catkin_ws_ov']
]
datasets = [
    ['euroc', ['V2_02_medium']]
    #['euroc', ['MH_01_easy', 'MH_02_easy', 'MH_03_medium', 'MH_04_difficult', 'MH_05_difficult', 'V1_01_easy', 'V1_02_medium', 'V1_03_difficult', 'V2_01_easy', 'V2_02_medium', 'V2_03_difficult']]
    # ['hilti', ['exp06_construction_upper_level_3', 'exp16_attic_to_upper_gallery_2', 'exp10_cupola_2', 'exp18_corridor_lower_gallery_2', 'exp14_basement_2']]
]
frame_rates = [1, .5, .75, .25]
multiple_repeat = 1
orbslam_types = [
    #"Stereo_Inertial", "Stereo", "Mono", "Mono_Inertial"
    "Mono"
]

# To loop through these, go down to the for loop at the bottom
slam_msckf_features = "max_slam:=50 max_slam_in_update:=25 max_msckf_in_update:=40"

def run(slam_msckf_features="", orbslam_type="Stereo_Inertial", prepend=""):
    for (dataset_name, all_seqs) in datasets:
        base_frame_rate = 1
        reroute_topics = ""

        if orbslam_type == "Stereo_Inertial":
            orbslam_calib_file_location = "Stereo-Inertial/EuRoC.yaml"
            do_rectify = "false"
        elif orbslam_type == "Mono_Inertial":
            orbslam_calib_file_location = "Monocular-Inertial/EuRoC.yaml"
            do_rectify = ""
        elif orbslam_type == "Stereo":
            orbslam_calib_file_location = "Stereo/EuRoC.yaml"
            do_rectify = "false"
        elif orbslam_type == "Mono":
            orbslam_calib_file_location = "Monocular/EuRoC.yaml"
            do_rectify = ""
        else:
            print("Unknown ORBSLAM type!")
            return

        for seq_name in all_seqs:
            short_seq_name = "".join(seq_name.split("_")[:2])

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
                            system_cmd = 'rosrun ORB_SLAM3 {} {}/Vocabulary/ORBvoc.txt {}/Examples/{} {}'.format(orbslam_type, system_dir, system_dir, orbslam_calib_file_location, do_rectify)
                            save_cmd = 'mv KeyFrameTrajectory_TUM_Format.txt FrameTrajectory_KITTI_Format.txt FrameTrajectory_TUM_Format.txt CameraTrajectory.txt KeyFrameTrajectory.txt output.txt {}'.format(final_results_dir)
                            #connectivity.txt connectivity_all.txt 
                        elif system_name == "kimera":
                            parallel = "parallel"
                            system_cmd = 'roslaunch kimera_vio_ros kimera_vio_ros_{}.launch log_output_path:={}/results'.format(dataset_name, system_dir)
                            save_cmd = 'mv {}/results/output_frontend_stats.csv {}/results/traj_vio.csv {}/results/traj_pgo.csv output.txt connectivity.dot connectivity_acceptable_factors.txt connectivity_lcd_factors.txt connectivity_rpgo.dot {}'.format(system_dir, system_dir, system_dir, final_results_dir)
                        elif system_name == "openvins":
                            parallel = "4threads"
                            system_cmd = "roslaunch ov_msckf subscribe.launch config:={}_mav dosave:=true path_est:={}/results/trajectory.txt dotime:=true path_time:={}/results/time_provided.txt {}".format(dataset_name, system_dir, system_dir, slam_msckf_features)
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



for orbslam_type in orbslam_types:
    print("!!!!!!!!!!!!!!!!!!!!!!!!", orbslam_type)
    run(slam_msckf_features, orbslam_type, "[{}]".format(orbslam_type))

# Below code is to run the msckf vs. slam features experiments for openvins
# for max_slam in range(0, 101, 25):
#     l = [int(max_slam/2), max_slam] if max_slam > 0 else [0]
#     for max_slam_in_update in l:
#         for max_msckf_in_update in range(0, 101, 25):
#             if max_slam == 0 and max_msckf_in_update == 0:
#                 continue
#             slam_msckf_features = "max_slam:={} max_slam_in_update:={} max_msckf_in_update:={}".format(max_slam, max_slam_in_update, max_msckf_in_update)
#             print(max_slam, max_slam_in_update, max_msckf_in_update)
#             prepend = "{}_{}_{}_".format(max_slam, max_slam_in_update, max_msckf_in_update)

#             run(slam_msckf_features)
