idf_component_register(SRCS "station_example_main.c" "chat.c" "sntp_example_main.c"
                    PRIV_REQUIRES onesdk
                    # INCLUDE_DIRS "../libwebsockets/include;${IDF_PATH}/components/spi_flash/include;${IDF_PATH}/components/nvs_flash/include")
                    INCLUDE_DIRS "${IDF_PATH}/components/spi_flash/include;${IDF_PATH}/components/nvs_flash/include")

# target_link_libraries(${COMPONENT_LIB} PRIVATE onesdk)
message(STATUS "onesdk_esp32 inlcude dir: ")
include_directories(../build/libwebsockets/include)