linux ubuntu 更新/安装clang编译器

[TOC]
==下述以`clang-10`更新为`clang-12`为例，安装同样步骤==

1. **更新apt**

```sh
sudo apt update
```

2. **查看`clang`版本**

```sh
clang --version
```

![alt text](image.png)

也可以运行以下命令查看指定版本的`clang`是否安装

```sh
dpkg -l | grep clang-12
```

- 我是以安装了clang-10为例，下面升级到clang-12为例

3. **安装`clang-12`**

```sh
sudo apt install clang-12
```

4. **查找正确的安装路径**
如果你已经安装了 `clang-12`，但不确定其安装路径，可以使用 `find` 或 `locate` 命令来查找它：

```sh
sudo find / -name clang-12
locate clang-12
```

安装后当你再次查看版本时还是显示的是`clang-10`，还需要更新`update-alternatives`

5. **使用正确的路径更新 `update-alternatives`**
假设你找到了 `clang-12` 的正确路径（例如 /usr/bin/clang-12），你可以使用以下命令来更新 `update-alternatives`

```sh
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-12 100
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-12 100
```

如果 `clang-12` 实际上安装在了不同的路径，你需要将上述命令中的 /usr/bin/clang-12 替换为实际路径。
6. **确认更新**
完成上述步骤后，再次确认`clang` 的版本：

```sh
clang --version
```

![alt text](image-1.png)

- 到这步，你已经成功更新`clang`的版本！
