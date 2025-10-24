cmake_minimum_required(VERSION 3.20.0)

set(_board_roots ${CMAKE_CURRENT_SOURCE_DIR})

set(_local_board_root ${CMAKE_CURRENT_SOURCE_DIR}/__boards__)
if(EXISTS ${_local_board_root})
  list(APPEND _board_roots ${_local_board_root})
endif()

if(DEFINED BOARD_ROOT)
  list(APPEND _board_roots ${BOARD_ROOT})
endif()

list(REMOVE_DUPLICATES _board_roots)
set(BOARD_ROOT "${_board_roots}" CACHE STRING "" FORCE)

string(REPLACE "/" "_" _board_overlay_name "${BOARD}")
set(_candidate_overlays
  ${CMAKE_CURRENT_SOURCE_DIR}/boards/${_board_overlay_name}.overlay
  ${CMAKE_CURRENT_SOURCE_DIR}/__boards__/${_board_overlay_name}.overlay
)

foreach(_overlay ${_candidate_overlays})
  if(EXISTS ${_overlay})
    if(DEFINED DTC_OVERLAY_FILE)
      list(APPEND DTC_OVERLAY_FILE ${_overlay})
    else()
      set(DTC_OVERLAY_FILE ${_overlay})
    endif()
  endif()
endforeach()

if(DEFINED DTC_OVERLAY_FILE)
  list(REMOVE_DUPLICATES DTC_OVERLAY_FILE)
  set(DTC_OVERLAY_FILE "${DTC_OVERLAY_FILE}" CACHE STRING "" FORCE)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(adxl366)

target_sources(app PRIVATE src/main.c)

target_include_directories(app PRIVATE
  ${ZEPHYR_BASE}/drivers/sensor/adi/adxl367
)
