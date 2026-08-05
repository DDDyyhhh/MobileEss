from pathlib import Path


FAST_LIO_ROOT = Path(__file__).resolve().parents[1]


def test_extrinsics_are_validated_before_eigen_array_expansion_and_logs_are_safe():
    source = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()
    guard = (FAST_LIO_ROOT / "include" / "mapping_guard.hpp").read_text()

    assert "valid_extrinsic_vector" in guard
    assert "values.size() != expected_size" in guard
    assert "std::isfinite(value)" in guard
    assert "const bool valid_translation" in source
    assert "const bool valid_rotation" in source
    assert "RCLCPP_FATAL" in source
    assert "numeric field-calibrated external" in source
    assert source.index("valid_extrinsic_vector(extrinT, 3)") < source.index("VEC_FROM_ARRAY(extrinT)")
    assert source.index("valid_extrinsic_vector(extrinR, 9)") < source.index("MAT_FROM_ARRAY(extrinR)")
    assert "FILE *fp = nullptr;" in source
    assert "if (fp == nullptr)" in source
    assert "if (fp != nullptr)" in source


def test_preprocess_rejects_out_of_range_pointcloud_rings_before_indexing():
    source = (FAST_LIO_ROOT / "src" / "preprocess.cpp").read_text()
    feature_disabled = source[
        source.index(
            "  else\n  {\n    for (int i = 0; i < plsize; i++)",
            source.index("void Preprocess::velodyne_handler"),
        ): source.index("void Preprocess::mid360_handler")
    ]
    assert "layer < 0 || layer >= N_SCANS" in feature_disabled


def test_preprocess_rejects_out_of_range_mid360_lines_before_indexing():
    source = (FAST_LIO_ROOT / "src" / "preprocess.cpp").read_text()
    handler = source[
        source.index("void Preprocess::mid360_handler"): source.index(
            "void Preprocess::default_handler"
        )
    ]
    assert "layer < 0 || layer >= N_SCANS" in handler


def test_mapping_resets_effective_points_before_matching():
    source = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()
    match = source[source.index("void h_share_model"): source.index("class LaserMappingNode")]
    assert "laserCloudOri->clear();" in match
    assert "corr_normvect->clear();" in match
    assert "total_residual = 0.0;" in match
    assert "effct_feat_num = 0;" in match


def test_mapping_skips_empty_correspondence_frames_before_publication():
    source = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()
    update = source[
        source.index("/*** iterated state estimation ***/"): source.index(
            "/******* Publish odometry *******/"
        )
    ]
    assert "effct_feat_num = 0;" in update
    assert "bool measurement_valid = false;" in update
    assert "&measurement_valid" in update
    assert "No effective correspondences; skipping this scan" in update
    assert "mapping_update_usable(measurement_valid, effct_feat_num)" in update


def test_mapping_propagates_ikfom_measurement_validity():
    source = (FAST_LIO_ROOT / "include/IKFoM_toolkit/esekfom/esekfom.hpp").read_text()
    assert "bool *measurement_valid_out = nullptr" in source
    assert "*measurement_valid_out = dyn_share.valid" in source


def test_preprocess_clamps_zero_filter_divisor():
    source = (FAST_LIO_ROOT / "src" / "preprocess.cpp").read_text()
    assert "point_filter_num = std::max(pfilt_num, 1);" in source


def test_feature_path_stops_when_all_points_are_blind():
    source = (FAST_LIO_ROOT / "src" / "preprocess.cpp").read_text()
    assert "while (head < types.size() && types[head].range < blind)" in source
    assert "if (head >= types.size())" in source




def test_mapping_uses_explicit_startup_state_and_counts_synchronized_packages():
    source = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()

    assert "bool   lidar_pushed = false" in source
    assert "flg_EKF_inited = false" in source
    assert "++sync_package_count;" in source


def test_odometry_populates_covariance_before_publication_and_records_evidence():
    source = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()
    publish = source[source.index("void publish_odometry"): source.index("void publish_path")]

    assert publish.index("odomAftMapped.pose.covariance") < publish.index("pubOdomAftMapped->publish(odomAftMapped);")
    assert "++odometry_publish_count;" in publish
    assert '"[T13-DIAG] odometry_published="' in publish


def test_imu_reset_initializes_startup_time_state():
    source = (FAST_LIO_ROOT / "src" / "IMU_Processing.hpp").read_text()

    assert "double first_lidar_time = 0.0;" in source
    assert "double last_lidar_end_time_ = 0.0;" in source
    reset = source[source.index("void ImuProcess::Reset()"): source.index("void ImuProcess::set_extrinsic")]
    assert "b_first_frame_   = true;" in reset
    assert "init_iter_num     = 1;" in reset
    assert "last_lidar_end_time_ = 0.0;" in reset


def test_mapping_can_emit_opt_in_correspondence_diagnostics():
    source = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()

    assert '"runtime_mapping_diagnostics"' in source
    assert '"[T13-DIAG] scan=' in source
    assert "candidate_count" in source
    assert "plane_fit_count" in source
    assert "residual_gate_count" in source
    assert "nearest_neighbor_requirement" in source
    assert "initialized_map_points" in source
    assert "initialized_tree_size" in source
    assert '"[T13-DIAG] sync_ready=%d' in source
    assert '"[T13-DIAG] startup=first_scan' in source
    assert '"[T13-DIAG] startup=post_imu_process' in source
    assert '"[T13-DIAG] startup=post_fov' in source
    assert "kdtree_delete_counter" in source
    assert "tree_root=%s" in source


def test_fov_deletion_requires_a_valid_local_map_cube_size():
    source = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()

    assert "cube_len <= 2.0 * MOV_THRESHOLD * DET_RANGE" in source
    assert "skipping FOV deletion" in source
    assert source.index("cube_len <= 2.0 * MOV_THRESHOLD * DET_RANGE") < source.index(
        "float dist_to_map_edge[3][2]"
    )


def test_imu_diagnostics_are_opt_in_and_expose_startup_boundaries():
    source = (FAST_LIO_ROOT / "src" / "IMU_Processing.hpp").read_text()

    assert "void set_diagnostics_enabled(bool enabled);" in source
    assert "void set_acc_bias_cov(const V3D &b_a);" in source
    assert "Eigen::Matrix<double, 12, 12> Q;" in source
    assert "bool diagnostics_enabled_ = false;" in source
    assert '"[T13-DIAG] imu_init iteration="' in source
    assert '"[T13-DIAG] imu_init_complete iteration="' in source


def test_replay_evidence_preserves_observer_endpoint_state_before_fastlio_stop():
    observer = (FAST_LIO_ROOT / "test" / "replay_odometry_observer.py").read_text()
    runner = (FAST_LIO_ROOT / "test" / "run_isolated_replay.sh").read_text()

    assert "max_publisher_endpoint_count" in observer
    assert "node._record_publishers()" in observer
    assert "wait \"$observer_pid\"" in runner
    assert "stop_process \"$lio_pid\"" in runner
    assert runner.index("wait \"$observer_pid\"") < runner.index(
        "stop_process \"$lio_pid\""
    )
    assert "fastlio_alive_before_observer_summary" in runner
    assert 'rosbag metadata missing: $bag_dir/metadata.yaml' in runner
    assert "rclpy.ok() else []" in observer
