cmake_minimum_required(VERSION 3.10)
project(ai_drone_gas)

# Sadece Python scriptlerini kuracağız
find_package(ament_cmake REQUIRED)

# scripts klasöründeki çalıştırılabilir dosyaları yükle
install(PROGRAMS
  scripts/sim_gas_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
