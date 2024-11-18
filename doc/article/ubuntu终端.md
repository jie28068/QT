linux终端美化[zsh]与一些效率工具使用
[toc]
### terminator
- **它允许用户在单个窗口中打开多个终端会话**
```sh
sudo apt update
sudo apt install terminator
```
![alt text](image-4.png)
### zsh
- **`zsh`是`shell`的替代品，它具有更好的功能和性能，并且可以轻松地配置**
```sh
sudo apt install zsh
```
- `Zsh` 的配置文件通常位于 **~/.zshrc**

#### 安装oh-my-zsh
- **用于管理 `Zsh` 配置。它通过提供插件、主题以及便捷的配置选项来增强 `Zsh` 的使用体验**
```sh
sh -c "$(curl -fsSL https://gitee.com/shmhlsy/oh-my-zsh-install.sh/raw/master/install.sh)"
```
或者
```sh
sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
```
#### 安装zsh常用插件
- 进入`oh-my-zsh`的`plugings`文件夹
```sh
cd ~/.oh-my-zsh/custom/plugins/
```
##### 安装zsh-autosuggestions
- **`zsh-autosuggestions`是一个zsh插件，它允许用户在输入命令时自动显示命令建议**
```sh
git clone https://github.com/zsh-users/zsh-autosuggestions
```
##### 安装zsh-syntax-highlighting
- **`zsh-syntax-highlighting`是一个zsh插件，它允许用户在输入命令时高亮显示命令的语法**
```sh
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git
```
##### 安装zsh-history-substring-search
- **`zsh-history-substring-search`是一个zsh插件，它允许用户在输入命令时使用上下键来浏览历史命令**
```sh
git clone https://github.com/zsh-users/zsh-history-substring-search.git
```

##### 配置zsh环境
1. 打开`~/.zshrc`文件,输入
```sh
code ~/.zshrc
```
2. 找到`plugins=()`
3. 然后将你在前面安装的插件添加到`plugins=()`中，例如：
```sh
plugins=(git zsh-autosuggestions zsh-syntax-highlighting zsh-history-substring-search)
```

### tldr
- **它提供了简洁的、可读的、可搜索的命令文档**
```sh
sudo apt install tldr
```
- 例如以下指令tldr 将返回一个简短的摘要，描述 ls 命令的用途和基本用法。
```sh
tldr ls
```
![alt text](image-5.png)

### ranger
- **它运行在终端环境中，提供了一个基于文本的用户界面（TUI）**
```sh
sudo apt update
sudo apt install ranger
```
- 使用`ranger`指令便可启动
![alt text](image-3.png)
- 常用指令
yy：复制文件或目录。
pp：粘贴文件或目录。
h/j/k/l：分别用于向左、向下、向上、向右移动。
gg：跳转到文件列表的顶部
...

### Lazygit
- **它运行在终端环境中，一个用于 Git 版本控制的终端界面（TUI）**
- 安装
```sh
sudo apt install golang
go get github.com/jesseduffield/lazygit
```
- 使用`Lazygit`指令便可启动


### glances
- **它运行在终端环境中，是一个用于监控系统资源的终端界面（TUI）**
```sh
sudo apt install glances
```
- 使用`glances`指令便可启动

![alt text](image-6.png)

### shc
- **一个用于加密和压缩shell脚本的工具。它通过加密脚本内容来防止直接查看脚本内容，从而增加了一层安全性**
```sh
sudo apt install shc
```

==上述的各种工具命令都可以通过`tldr`指令来简单查看==

~~待补充~~