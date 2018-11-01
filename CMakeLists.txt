cmake_minimum_required(VERSION 2.8.3)
project(odom_extrinsic_calibrate)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic -Werror=return-type")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package( Ceres 1.6.0 QUIET )
MESSAGE(STATUS "CERES found")


include_directories(include
thirdpart
${CERES_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
      src/calibrate_process.cpp
      #src/calibrEncoderOnline.cpp
		src/OdomCalibrateError.cpp
		src/OdomSensorExtrinsicError.cpp
		src/OdomExtrinsicCalibrateError.cpp
		src/odom_calibrate_solver.cpp
		src/utils.cpp
		src/Time.cpp
		src/Duration.cpp
        )
target_link_libraries(${PROJECT_NAME}
${CERES_LIBRARIES} pthread)



	 add_executable(calib
	 	sample/calibrate_encoder.cpp
	 	sample/utils/read_dataset.cpp
	 	sample/utils/read_dataset.h
	 		)
	 target_link_libraries(calib
	 		${PROJECT_NAME}
	 		)


	 add_executable(testOdomExtrinsicCalibrateError
	 		test/testOdomExtrinsicCalibrateError.cpp
	 		)
	 target_link_libraries(testOdomExtrinsicCalibrateError
	 		${PROJECT_NAME}
	 		)

	 add_executable(testOdomSensorExtrinsic
	 		test/testOdomSensorExtrinsic.cpp
	 		)
	 target_link_libraries(testOdomSensorExtrinsic
	 		${PROJECT_NAME}
	 		)


     add_executable(testOdomCalibrateExtrinsicSimulation
	 		test/testOdomCalibrateExtrinsicSimulation.cpp
	 		)
	 target_link_libraries(testOdomCalibrateExtrinsicSimulation
	 		${PROJECT_NAME}
	 		)


	 add_executable(testOdomSensorExtrinsicSimulation
	 		test/testOdomSensorExtrinsicSimulation.cpp
	 		)
	 target_link_libraries(testOdomSensorExtrinsicSimulation
	 		${PROJECT_NAME}
	 		)


	 add_executable(test_jacobian2 test/test_jacobian2.cpp)
	 target_link_libraries(test_jacobian2 ${PROJECT_NAME})


