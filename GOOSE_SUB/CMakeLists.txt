
set(goose_sub_gpio_SRCS
   goose_sub_gpio.c
)

IF(MSVC)

set_source_files_properties(${goose_sub_gpio_SRCS}
                                       PROPERTIES LANGUAGE CXX)
ENDIF(MSVC)                                       
                                       
add_executable(goose_publisher_example
  ${goose_sub_gpio_SRCS}
)

target_link_libraries(goose_publisher_example
    iec61850
)


