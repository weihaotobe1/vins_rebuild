add_library(${PROJECT_NAME} SHARED

        # camera model
        camera_model/calib/CameraCalibration.cc # calib
        camera_model/camera_models/Camera.cc    # camera_models
        camera_model/camera_models/CameraFactory.cc
        camera_model/camera_models/CostFunctionFactory.cc
        camera_model/camera_models/PinholeCamera.cc
        camera_model/camera_models/CataCamera.cc
        camera_model/camera_models/EquidistantCamera.cc
        camera_model/camera_models/ScaramuzzaCamera.cc
        camera_model/sparse_graph/Transform.cc   # spares_graph
        camera_model/gpl/gpl.cc                  # gpl
        camera_model/gpl/EigenQuaternionParameterization.cc


        # factor
        factor/marginalization_factor.cpp
        factor/pose_local_parameterization.cpp
        factor/projection_factor.cpp
        factor/projection_td_factor.cpp

        #initial
        initial/initial_alignment.cpp
        initial/initial_ex_rotation.cpp
        initial/initial_sfm.cpp
        initial/solve_5pts.cpp

        # feature tracker
        feature_manager.cpp
        feature_tracker.cpp
        featuretrackerparameters.cpp

        # vins core 
        estimator.cpp
        estimatorparameters.cpp
        system.cpp

        # utility
        visualization2.cpp
        
        
        # pose graph
        pose_graph/DBoW/BowVector.cpp #DBOW
        pose_graph/DBoW/FBrief.cpp
        pose_graph/DBoW/FeatureVector.cpp
        pose_graph/DBoW/QueryResults.cpp
        pose_graph/DBoW/ScoringObject.cpp
        pose_graph/DUtils/Random.cpp  # DUtils
        pose_graph/DUtils/Timestamp.cpp
        pose_graph/DVision/BRIEF.cpp  #DVision

        pose_graph/keyframe.cpp
        pose_graph/MapDrawer.cpp
        pose_graph/pgsystem.cpp
        pose_graph/pose_graph.cpp
        pose_graph/posegraphparameters.cpp
        pose_graph/utility.cpp
        pose_graph/VocabularyBinary.cpp

)
target_link_libraries(${PROJECT_NAME} ${ThirdParty_Libraries})