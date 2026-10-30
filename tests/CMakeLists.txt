add_library(test_main OBJECT test_main.cpp)
target_link_libraries(test_main PUBLIC mavgs_options)

function(mavgs_add_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE mavgs test_main)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

mavgs_add_test(test_frame)
mavgs_add_test(test_messages)
mavgs_add_test(test_mission)
mavgs_add_test(test_stats_tlog_link)
mavgs_add_test(test_sim)
mavgs_add_test(test_station)
mavgs_add_test(test_cli_script)

if(MAVGS_BUILD_APPS)
  find_program(BASH_PROGRAM bash)
  if(BASH_PROGRAM)
    add_test(NAME e2e_udp
             COMMAND ${BASH_PROGRAM} ${CMAKE_CURRENT_SOURCE_DIR}/e2e_udp.sh
                     $<TARGET_FILE_DIR:mavgcs> ${PROJECT_SOURCE_DIR})
    set_tests_properties(e2e_udp PROPERTIES TIMEOUT 120)
  endif()
endif()
