cmake_minimum_required(VERSION 3.8)

project(r2_robotic_arm_description)

find_package(ament_cmake REQUIRED)

foreach(dir config launch meshes urdf rviz)
    install(DIRECTORY ${dir}/
        DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

ament_package()
