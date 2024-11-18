## 命令

### project

- **用于定义一个项目，它是 CMakeLists.txt 文件中的第一个命令，通常也是必须的命令。project 命令的主要作用是设置项目的名称，并且可以指定项目的版本和语言。**
- ex

    ```c
    project(MyProject VERSION 1.0.0 LANGUAGES CXX)
    ```

- 后续使用项目名MyProject时可以直接使用宏`${PROJECT_NAME}`替换

### file

- **用于执行各种文件和目录相关的操作。它可以用来读写文件、创建目录、删除文件、获取文件属性等**
    1. file(READ)：读取文件内容到一个变量中。
    2. file(WRITE)：将内容写入到一个文件中。
    3. file(APPEND)：将内容追加到一个文件的末尾。
    4. file(GLOB)：搜索匹配特定模式的文件，并将结果存储在一个变量中。
    5. file(MAKE_DIRECTORY)：创建一个目录。
    6. file(REMOVE)：删除文件。
    7. file(COPY)：复制文件或目录。
    8. file(REAL_PATH)：获取文件的绝对路径

### set

- **用于设置一个变量的值。这个变量可以是普通的变量、缓存变量或者环境变量**

### configure_file

- **通常用于生成配置头文件、资源文件或者是一些根据构建系统变量动态生成的文件。**
- 例如，如果你有一个名为 config.h.in 的模板文件，其中包含了如下内容：

```c
#define PROJECT_VERSION_MAJOR @PROJECT_VERSION_MAJOR@
#define PROJECT_VERSION_MINOR @PROJECT_VERSION_MINOR@
```

```c
project(MyProject VERSION 1.0)
configure_file(config.h.in config.h)
```

在生成 config.h 文件时，@PROJECT_VERSION_MAJOR@ 和 @PROJECT_VERSION_MINOR@ 会被替换为项目的版本号中的主要和次要版本号。

### list

- **用于操作列表类型的变量**
    1. LENGTH：获取列表的长度。
    2. APPEND：向列表末尾添加一个或多个元素。
    3. FIND：在列表中查找一个元素的索引。
    4. ...

### add_library

- **用于定义一个库目标，这个库目标可以是静态库、共享库或者是一个对象库。定义库目标后，CMake 会生成相应的构建规则，以便在构建过程中编译库的源代码。**
- 在定义库目标之后，你可以使用 `target_link_libraries` 命令来指定库应该链接的其他库，使用 `target_include_directories` 来指定库的包含目录，以及其他用于配置目标属性的命令。

```c
add_library(<name> [STATIC | SHARED | MODULE]
             [EXCLUDE_FROM_ALL]
             [source1] [source2] [...])
```

- name：指定库目标的名称。这个名称用于在 CMake 项目中引用库目标。
- [STATIC | SHARED | MODULE]：指定库的类型。STATIC 表示静态库，SHARED 表示共享库（动态链接库），MODULE 表示不会被直接链接的模块，通常用于插件。**如果省略这个参数，CMake 会根据变量 ==BUILD_SHARED_LIBS== 的值决定是生成静态库还是共享库**。
- [EXCLUDE_FROM_ALL]：如果指定了这个选项，那么这个库目标不会默认被构建，除非明确指定要构建它。
- [source1] [source2] [...]：指定库的源代码文件。这些文件将会被编译并包含在库中。

### set_target_properties

- **用于设置一个目标的属性，这些属性可以控制目标的构建行为，比如编译选项、链接选项、包含目录、输出目录等**
- ex

    ```c
        set_target_properties(MyExecutable PROPERTIES
            OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
            COMPILE_FLAGS "-Wall -Wextra")
    ```

- 在这个例子中，`OUTPUT_DIRECTORY` 属性被设置为构建目录下的 bin 子目录，而 `COMPILE_FLAGS` 属性被用来添加额外的编译器警告标志。

### target_link_directories

- **用于指定目标链接时搜索库文件的目录的命令。这个命令告诉 CMake 当编译一个目标（比如可执行文件或库）时，到哪里查找需要的库文件。**

```c
target_link_directories(<target> [BEFORE]
  <INTERFACE|PUBLIC|PRIVATE> [items1...]
  ...
)
```

- <target>：指定要添加链接目录的目标，这个目标必须是之前通过 `add_executable()` 或 `add_library()` 创建的。
- [BEFORE]：可选参数，如果指定，这些目录将在目标自己的链接目录之前被搜索。
- <INTERFACE|PUBLIC|PRIVATE>：指定链接目录的作用域。
  - **INTERFACE**：目录只会影响那些链接到目标的其他目标。
  - **PUBLIC**：目录会影响目标本身和链接到目标的其他目标。
  - **PRIVATE**：目录只会影响目标本身
- [items1...]：要添加的目录列表。

### target_link_libraries

- **用于指定目标（如可执行文件或库）链接时需要的库。这个命令告诉 CMake 在构建一个目标时，需要链接哪些库，以及这些库的链接顺序。**
- 在链接库时，库的顺序很重要，特别是当链接器依赖于库的依赖关系顺序时。CMake 会按照你指定的顺序将库传递给链接器。
- ex

    ```c
    target_link_libraries(MyExecutable Qt5::Gui Qt5::Widget)
    ```

### target_compile_definitions

- **用于为**特定**的目标（如可执行文件或库）添加或取消编译定义。编译定义通常用于条件编译，它们可以在源代码中通过预处理器指令（如 #ifdef）来检测。**

### add_subdirectory

- **用于将一个子目录添加到构建树中。这个命令会使得 CMake 处理子目录中的 CMakeLists.txt 文件，并添加其中的构建目标到当前项目的构建系统中**

### function

- **用于定义一个自定义函数，这样你就可以在 CMake 脚本中重复使用一段代码**
- ex 如果你想定义一个函数来计算两个数的和，并返回结果

```c
function(calculate_sum a b output)
  math(EXPR sum "${a} +${b}")
  set(${output}${sum} PARENT_SCOPE)
endfunction()

# 调用自定义函数并获取返回值
calculate_sum(5 3 result)
message(STATUS "The sum is: ${result}")

```

在这个例子中，calculate_sum 函数接受两个输入参数 a 和 b，以及一个输出参数 output。函数内部使用 math(EXPR) 命令计算两数之和，并使用 set 命令将结果存储在 output 参数中。PARENT_SCOPE 选项确保输出参数的值在函数外部可见。
