cmake_minimum_required(VERSION 3.0.2)

project(fourier)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory("${CMAKE_SOURCE_DIR}/openal")

add_executable(fourier
	"${CMAKE_SOURCE_DIR}/main.cpp"
	"${CMAKE_SOURCE_DIR}/openal/examples/common/alhelpers.c"
)
target_include_directories(fourier PRIVATE
	"${CMAKE_SOURCE_DIR}/openal/include"
	"${CMAKE_SOURCE_DIR}/openal/examples"
)
target_link_libraries(fourier OpenAL)

add_executable(piano
	"${CMAKE_SOURCE_DIR}/piano.cpp"
	"${CMAKE_SOURCE_DIR}/openal/examples/common/alhelpers.c"
)
target_include_directories(piano PRIVATE
	"${CMAKE_SOURCE_DIR}/openal/include"
	"${CMAKE_SOURCE_DIR}/openal/examples"
)
target_link_libraries(piano OpenAL)