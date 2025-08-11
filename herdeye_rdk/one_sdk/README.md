
# libwebsockets 相关编译命令

目前，默认是使用构建机器的openssl支持tls相关能力，mac上一般路径为：`/opt/homebrew/include`
可以在CMakeLists.txt中添加如下代码（按需修改路径）：
```cmake
	include_directories("/opt/homebrew/include") # openssl
```

1. 构建全量onesdk
```bash
bash build.sh
```


2. 不构建http2相关依赖
```bash
# 不构建http2相关依赖
bash build.sh -DLWS_WITH_HTTP2=0 -DLWS_ROLE_H2=0
```

3. 构建onesdk测试代码
```bash
# 构建测试代码，当前也支持在mac上构建和运行
cd examples/linux/onesdk_hello_world

bash build.sh
```
4.  配置 CMake 并禁用libwebsockets测试用例
在构建时，通过 CMake 参数 -DLWS_WITHOUT_TESTAPPS=ON 来禁用测试用例的编译。
其他常见的 CMake 选项：

-DLWS_WITHOUT_EXTENSIONS=ON：禁用扩展支持。

-DLWS_WITHOUT_SERVER=ON：禁用服务器功能。

-DLWS_WITHOUT_CLIENT=ON：禁用客户端功能。
```bash

bash build.sh -DLWS_WITHOUT_TESTAPPS=ON -DLWS_WITHOUT_TEST_CLIENT=ON -DLWS_WITHOUT_TEST_PING=ON
```

## esp32支持

参考 [README.md](examples/onesdk-esp32/README.md)
