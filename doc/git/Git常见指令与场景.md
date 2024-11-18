# 一文带你理解完Git知识点

## Git基础概念

Git 是一个分布式版本控制系统，用于追踪文件的更改和协助多人合作开发。在 Git 中，有几个核心概念需要理解，包括工作区（Workspace）、暂存区（Staging Area，也称作索引 Index）和版本库（Repository）。

1. **工作区（Workspace）**：
   工作区是指你在电脑上看到的文件目录，是你直接编辑文件的地方。在 Git 管理的目录中，你的文件会直接被 Git 跟踪，除非你明确告诉 Git 忽略它们（通过 .gitignore 文件）。工作区中的文件状态可以是已修改（modified）、已暂存（staged）或已提交（committed）。
2. **暂存区（Staging Area）**：
   暂存区是一个文件，保存了即将进行提交的变更快照。它是工作区和版本库之间的缓冲区。暂存区的存在使得开发者可以精细控制哪些改动会被包含在下一个提交中。你可以将工作区的改动添加到暂存区（通过 `git add` 命令），也可以从暂存区移除（通过 `git reset HEAD <file>` 命令）。暂存区使得提交更加可控和精细。
3. **版本库（Repository）**：
   版本库是 Git 用来保存项目历史记录的地方，其中包含了所有文件的版本和历史信息。版本库位于工作区的 .git 目录中。当你进行提交操作时（通过 `git commit` 命令），Git 会将暂存区的当前状态永久保存在版本库中。这样，你就可以通过版本库回溯到任何一个历史版本。

----
简单来说，你的工作流程大致如下：

- 在工作区中修改文件。
- 使用 `git add` 命令将改动添加到暂存区。
- 使用 `git commit` 命令将暂存区的改动提交到版本库。
通过这种方式，Git 提供了一个有效的机制来管理文件的修改历史，支持多人协作，并且可以方便地撤销错误的改动。

下面这个图展示了工作区、版本库中的暂存区和版本库之间的关系：

1. 图中左侧为工作区，右侧为版本库。在版本库中标记为 "index" 的区域是暂存区（stage/index），标记为 "master" 的是 master 分支所代表的目录树。

2. 图中我们可以看出此时 "HEAD" 实际是指向 master 分支的一个"游标"。所以图示的命令中出现 HEAD 的地方可以用 master 来替换。

3. 图中的 objects 标识的区域为 Git 的对象库，实际位于 ".git/objects" 目录下，里面包含了创建的各种对象及内容。

4. 当对工作区修改（或新增）的文件执行 git add 命令时，暂存区的目录树被更新，同时工作区修改（或新增）的文件内容被写入到对象库中的一个新的对象中，而该对象的ID被记录在暂存区的文件索引中。

5. 当执行提交操作（`git commit`）时，暂存区的目录树写到版本库（对象库）中，master 分支会做相应的更新。即 master 指向的目录树就是提交时暂存区的目录树。

6. 当执行 `git reset HEAD` 命令时，暂存区的目录树会被重写，被 master 分支指向的目录树所替换，但是工作区不受影响。

7. 当执行 `git rm --cached <file>` 命令时，会直接从暂存区删除文件，工作区则不做出改变。

8. 当执行 `git checkout .` 或者 `git checkout -- <file>` 命令时，会用暂存区全部或指定的文件替换工作区的文件。这个操作很危险，会清除工作区中未添加到暂存区中的改动。

9. 当执行 `git checkout HEAD .` 或者 `git checkout HEAD <file>` 命令时，会用 HEAD 指向的 master 分支中的全部或者部分文件替换暂存区和以及工作区中的文件。**这个命令也是极具危险性的**，因为不但会清除工作区中未提交的改动，也会清除暂存区中未提交的改动。

---

## Git基本操作

Git 常用的是以下 6 个命令：**git clone、git add 、git commit、git push、git pull。**

**说明：**

- workspace：工作区
- staging area：暂存区/缓存区
- local repository：版本库或本地仓库
- remote repository：远程仓库

### **0. 初始化仓库**

```bash
git init
```

git仓库就创建好了，这时文件夹中多了一个.git文件夹。注意不要修改这个文件夹中的内容。  

### **1. add到暂存区**  

工作区有了修改后

```bash
git add xxx_file   //添加某个文件到暂存区,多个文件用空格分开
git add .         //添加所有文件到暂存区。
```

### **2. 再commit到本地仓库**

```bash
git commit -m "这里写注释"         //注释很重要，提交暂存区的所有内容
git commit 文件1 文件2 ... -m "这里写注释" //只提交暂存区的某些文件
```

### **3. 推送到远程仓库**

> git push <远程主机名> <本地分支名>:<远程分支名>

如果本地分支名与远程分支名相同，则可以省略冒号：

> git push <远程主机名> <本地分支名>

以下命令将本地的 master 分支推送到 origin 主机的 master 分支。

```bash
git push origin master
```

**注**：如果是首次push到远程仓库，则需要先添加远程仓库，执行命令 `git remote add [shortname] [url]`
例如：`git remote add origin https://github.com/xxx/xxx.git`

### **4. 拉取远程仓库**

```bash
git pull
```

### **5. 撤销更改**

- 撤销工作区的更改：`git checkout -- <file>`
- 撤销暂存区的更改：`git reset HEAD <file>`
- 撤销提交：`git commit --amend`（如果尚未推送）或使用 `git revert <commit>`（如果已经推送）

---

## Git分支管理

### 1. 创建分支命令

```bash
git branch branchname
```

### 2. 切换分支命令

```bash
git checkout branchname
```

或

```bash
git switch branchname
```

### 3. 摘取提交

选择一个或多个提交并将其应用到其他分支上

```bash
git cherry-pick <commit-hash>
```

### 4. 删除分支命令

```bash
git branch -d branchname
```

强制删除需使用`-D`

### 5. 合并分支命令

```bash
git merge dev //表示在当前分支合并dev分支
```

### 6. 变基

多分支开始时需要使用到。将一个分支的更改重新应用到另一个分支上，以保持提交历史的线性。

```bash
git rebase <base-branch>
```

在变基过程中解决冲突后继续

```bash
git add <resolved-file>
git rebase --continue
```

取消变基操作

```bash
git rebase --abort
```

==分支操作如果有冲突的话，个人习惯用小乌龟解决，图形化界面还是比指令来的直观==

---

## Git进阶

除了一些基本的常用命令，还有一些命令也是比较实用但可能被我们忽略的，比如`git tag、git reset、git stash`等。

### 1. **git tag 标签**

- 如果你达到一个重要的阶段，并希望永远记住那个特别的提交快照，你可以使用 git tag 给它打上标签。常用于项目版本发布节点；
- 比如说，想为我们项目发布一个"1.0"版本。 我们可以用 `git tag -a v1.0 -m "1.0版本发布"` 命令给最新一次提交打上"v1.0"的标签；
- Git 支持两种标签：轻量标签（lightweight）与附注标签（annotated）。`-a 选项`意为"创建一个带注解的标签"。 不用 -a 选项也可以执行的，但它不会记录这标签是啥时候打的，谁打的，也不会让你添加个标签的注解。 推荐使用带注解的标签；
- 通过使用 `git tag` 命令，可以查看当前有哪些标签；
- 通过使用 `git show` 命令可以看到标签信息和与之对应的提交信息；
- 后期打标签 `git tag -a v1.2 888888 -m "my tag"` 888888 为commit记录的ID号；
- 将标签同步到远程服务器。默认情况下，git push 命令并不会传送标签到远程仓库服务器上。 在创建完标签后你必须显式地推送标签到服务器上。 这个过程和推送代码一样——你可以运行 `git push origin v1.0` 。如果要推送本地所有tag，使用`git push origin --tags`；
- 删除某个标签。要删除掉你本地仓库上的标签，可以使用命令 `git tag -d v1.0`。该命令不会删除远程仓库标签，你必须用 `git push origin :refs/tags/v1.0` 来更新你的远程仓库，意思是，将冒号前面的空值推送到远程标签名，从而高效地删除它。第二种更直观的删除远程标签的方式是：`git push origin --delete v1.0`；

### 2. **git reset 版本回退**

- 有两种常用情况需要用到版本回退。第一种：觉得当前代码改乱了，想要恢复到之前某个版本；第二种：当程序出现了某个bug，但工程太大实在不好排查，用二分查找法回退到指定版本看是否重现，然后定位问题（这个方法看起来笨，但某些时候非常有效！）；
- 首先使用 `git log` 命令查看推送记录，这样便能看到每一次commit的id，然后使用 `git reset --hard 1234321` 命令回退到指定版本。（小tips：如果你当下有部分新增的代码片段你觉得还有用，那么最好拷贝出来。因为该操作会完全覆盖掉你当前的文件）；
- 如果再想恢复到之后的版本，log里是没有了，使用 `git reflog` 可以查到；

### 3. **git stash 贮藏**

有时，当你在项目的一部分上已经工作一段时间后，所有东西都进入了混乱的状态， 而这时你想要切换到另一个分支做一点别的事情。 问题是，现在的代码改得很混乱，甚至都无法编译通过，你并不想提交。这时就可以使用git stash命令。

- `git stash` 保存当前工作进度，会把暂存区和工作区的改动保存起来。使用 `git stash save "message"` 可以添加一些注释；
- 执行完这个命令后，再运行`git status`命令，就会发现当前是一个干净的工作区，没有任何改动；
- `git stash list` 显示保存进度的列表。也就是说，git stash命令可以多次执行；
>
- `git stash pop` 将缓存堆栈中的第一个stash删除，并将对应修改应用到当前的工作目录下；
- `git stash drop [stash_id]` 删除一个存储的进度。如果不指定stash_id，则默认删除最新的存储进度；
- `git stash clear` 删除所有存储的进度；

## Git高级

### **1. 子模块**

Git 子模块（Submodules）是 Git 中用于将一个 Git 仓库嵌入到另一个 Git 仓库中的功能。这通常用于以下情况：

- 当你想要在多个项目中使用同一个库，并且希望这个库保持独立的版本控制时。
- 当你不想将第三方库的源代码直接复制到你的项目中，而是希望以某种方式引用它时。
子模块允许你将一个 Git 仓库作为另一个仓库的子目录，同时保持其独立的提交历史。这意味着子模块可以有独立的提交、分支和标签，而主项目可以跟踪子模块的具体提交。
- **添加子模块**：

  ```bash
  git submodule add <repository-url> <path>
  ```

  这个命令会在指定的 `<path>` 处添加子模块，并将其注册到主项目的 `.gitmodules` 文件中。
- **克隆带有子模块的项目**：

  ```bash
  git clone <repository-url>
  cd <repository-name>
  git submodule init
  git submodule update
  ```

  克隆主项目后，需要初始化并更新子模块。
- **更新子模块**：

  ```bash
  git submodule update --init --recursive
  ```

  这个命令会拉取所有子模块的最新提交。
- **同步子模块的远程跟踪分支**：

  ```bash
  git submodule foreach git fetch
  git submodule foreach git merge origin/master
  ```

  这个命令会遍历所有子模块，执行 `fetch` 和 `merge` 操作。
- **删除子模块**：
  删除子模块比较复杂，需要从主项目的多个地方删除子模块的记录：

  ```bash
  git submodule deinit <submodule-path>
  rm -rf .git/modules/<submodule-path>
  git rm -f <submodule-path>
  rm .gitmodules
  ```

然后手动编辑 `.gitmodules` 文件，移除子模块的相关条目，最后提交这些更改。

### **2. 钩子**

Git 钩子（Hooks）是 Git 仓库中特定事件触发时自动运行的脚本。它们存在于每个 Git 仓库的 `.git/hooks` 目录中，Git 提供了一些预定义的钩子模板，你可以根据需要启用和自定义这些脚本。
钩子分为客户端钩子和服务器端钩子：

1. **客户端钩子**：
   - `pre-commit`：在 `git commit` 执行前触发，可以用来检查代码风格、运行测试等。
   - `prepare-commit-msg`：在编辑器打开提交信息之前触发，可以用来预填充提交信息。
   - `commit-msg`：在提交信息被提交前触发，可以用来验证提交信息格式。
   - `post-commit`：在提交完成后触发，通常用于通知或日志记录。
   - `pre-rebase`、`post-checkout`、`post-merge` 等：在 rebase、checkout 或 merge 操作前后触发。
2. **服务器端钩子**：
   - `pre-receive`：在服务器接收到推送的数据但还未更新引用时触发，可以用来拒绝不满足某些条件的推送。
   - `update`：与 `pre-receive` 类似，但是为每个更新的分支/引用触发一次。
   - `post-receive`：在所有更新完成后触发，常用于部署或邮件通知。
3. **启用钩子**：
   - 进入 `.git/hooks` 目录，你可以看到一些以 `.sample` 结尾的钩子模板文件。
   - 要启用一个钩子，你可以移除 `.sample` 扩展名，例如将 `pre-commit.sample` 重命名为 `pre-commit`。
   - 钩子脚本可以是任何可执行脚本，比如 Bash 脚本、Python 脚本等。

- 示例：pre-commit 钩子
以下是一个简单的 `pre-commit` 钩子示例，用于检查提交消息中是否包含 Issue 编号：

```bash
#!/bin/bash
# 获取提交消息
commit_msg=$(cat $1)
# 检查提交消息中是否包含 'Issue #'
if ! echo "$commit_msg" | grep -q "Issue #"; then
    echo "Commit message must contain 'Issue #' reference."
    exit 1
fi
# 如果一切正常，返回 0
exit 0
```

在这个例子中，如果提交消息中没有包含 "Issue #"，钩子会拒绝提交，并显示一条消息。如果一切正常，钩子会返回 0，允许提交继续进行。
Git 钩子是一个非常强大的工具，可以用来实施代码质量标准、自动化工作流程和增强团队协作。然而，使用钩子时应该谨慎，以避免过度复杂化开发流程。

## 总结

- 在实际开发过程你能理解下图，Git使用场景的差不多就这些乐。

- **个人觉得在你理解了Git的原理后，还是使用图形化界面操作方便点。。。**
