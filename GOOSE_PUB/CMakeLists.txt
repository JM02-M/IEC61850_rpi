
set(goose_pub_gpio_SRCS
   goose_pub_gpio.c
)

IF(MSVC)

set_source_files_properties(${goose_pub_gpio_SRCS}
                                       PROPERTIES LANGUAGE CXX)
ENDIF(MSVC)                                       
                                       
add_executable(goose_pub_gpio
  ${goose_pub_gpio_SRCS}
)

target_link_libraries(goose_publisher_example
    iec61850
)


