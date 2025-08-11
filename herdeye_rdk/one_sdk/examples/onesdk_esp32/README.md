## build

### Step 1: setup esp-idf environment
1. [esp-idf installation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html#get-started-prerequisites)
2. enable esp-idf environment in current shell
```
. ~/esp/esp-idf/export.sh
```

### Step 2: extract source code for esp32

> execute in onesdk root dirctory
> by default, the source will be extracted to `./build/outpout`.
> The `ONESDK_EXTRACT_SRC` is ON by default

```bash
bash build.sh -DONESDK_EXTRACT_SRC=ON
```

here is the layout of extracted source

```bash
.
├── build
│   ├── output
│   │   ├── libwebsockets
│   │   │   ├── include
│   │   │   ├── src
│   │   │   └── tests
│   │   ├── components
│   │   │   ├── onesdk`
```

### Step 3: copy sources(`libwebsockes` and `components`) to esp32 project root

Below is the example of esp32 project structure

```bash
✗ tree  -L 2
.
├── CMakeLists.txt
├── README.md
├── components
│   └── onesdk
├── libwebsockets
│   ├── ...
│   └── ...
├── main
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild
│   ├── chat.c
│   ├── chat.h
│   └── station_example_main.c
└── sdkconfig.defaults
```

### Step 4: build esp32 project

#### create a CMakeLists.txt in components/onesdk
> this will build onesdk as esp32 component

```cmake
file(GLOB_RECURSE SRC_FILES "src/*.c" "platform/*.c", "external_libs/cJson/cJSON*.c")

idf_component_register(SRCS ${SRC_FILES}
                    REQUIRES spi_flash esp_netif nvs_flash esp_wifi esp_driver_gpio mbedtls
                    INCLUDE_DIRS "." "include" "platform/include" "src" "external_libs/cjson" 
                    )
# link with libwebsockets starts
# target_link_libraries(${COMPONENT_LIB} PRIVATE websockets)
message(STATUS "${COMPONENT_LIB}: libwebsockets build directory: ${PROJECT_BINARY_DIR}/../libwebsockets/include")
target_include_directories(${COMPONENT_LIB} PRIVATE "${PROJECT_BINARY_DIR}/../libwebsockets/include")

# link with libwebsockets ends
```
#### apply libwebsockets changes in CMakeLists.txt and build

the patch file is `libwebsockets.diff`

```bash
cd examples/onesdk_esp32
. ~/esp/esp-idf/export.sh
#git apply libwebsockets.diff
idf.py build
idf.py flash -p </dev/your_device_name> monitor
```