# Ciallo~⭐

小登们好！这是你们的小组仓库，以后你们三人协作的代码可以上传在此。仓库名目前是GroupX，请自行更改为你们的队伍名。还有仓库描述，也自己随便写写。

> 读完此README.md**并理解Git用法**后，修改此README.md，把所有内容删掉然后写上你们仓库的目录结构和你们三个人的信息。

> 是的，学会Markdown基础语法也是目标之一

## 什么是 Git？为什么需要它？

**Git** 是一个“版本控制系统”。它大概就是一个代码的“时光机 + 协同办公软件”：

- **记录每一次改动**：你可以随时回看谁、什么时候、改了什么，再也不怕改崩了找不回来。
- **多人同时写代码不打架**：Git 能自动合并大部分修改；万一冲突了，也会清楚告诉你哪里需要手动商量。
- **备份在云端（GitHub）**：你们小组的仓库存在 GitHub 上，换电脑也能拉取下来继续写。

所以，别把 Git 当成多余的麻烦——它是你们三人协作的好工具。

## Git 相关操作

你目前大抵有两种方法使用Git：

- 通过命令行使用Git
- 通过图形界面工具使用Git (如GitHub Desktop)，这样更简单，适合初学者。但是等到以后你们开始接触linux等环境，命令行操作会是刚需。

或者你也可以混用这两种方式。

B站上面关于Git的教程也是一大堆，肯定能学会的。

### 命令行

#### 第一次使用

请遵循以下步骤：

1. 下载Git。[Git官方安装链接](https://git-scm.com/install/windows)。安装步骤请参考[Git 详细安装教程](https://blog.csdn.net/mukes/article/details/115693833)
2. 打开cmd或PS，输入```git --version```，如果显示```git version 2.53.x.xxx.x```说明已经安装成功了
3. 第一次使用，告诉 Git 你是谁（只需第一次使用时做）:
```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
```
注意这个邮箱请使用你github的绑定邮箱。这俩信息会出现在每一次提交记录里，别乱写。
4. 把小组仓库克隆到本地（每人只做一次）
```bash
git clone 仓库地址
```
仓库地址可以在GitHub上找到，点击绿色的Code按钮，复制HTTPS链接即可。

#### 日常使用

常用命令：
```bash
# 最常见的工作流程：
# 记得终端要位于你克隆下来的仓库目录里 (cd 你的仓库目录)

git pull                 # 1️⃣ 拉取队友最新代码（每次干活前先做）
# …… 修改代码，写石山(bushi) ……
git add .                # 2️⃣ 把改动加入“暂存区”（. 表示全部文件）
git commit -m "(此处写清楚你做了什么改动增删)"   # 3️⃣ 本地提交
git push                 # 4️⃣ 推送到 GitHub 上，队友才能看到
```

其他命令：
```bash
git status                      # 查看当前文件状态（哪些改了，哪些还没add）
git log                         # 查看提交历史
git diff                        # 查看改动内容
git checkout -- 文件名           # 放弃对某个文件的改动，恢复到上次提交的状态
git reset                       # 撤销上一次git add操作
git commit --amend -m "(新说明)" # 修改上一次提交的说明（如果你刚提交了但发现说明写错了，可以用这个命令修改）

# 更多指令用法请参考其他教程
```
### 图形界面工具

以 GitHub Desktop 为例。

#### 第一次使用

1. 下载 GitHub Desktop。[GitHub Desktop下载链接](https://github.com/apps/desktop)
2. 安装完成后，打开GitHub Desktop，登录你的GitHub账号。
3. 在GitHub Desktop中，点击File -> Clone Repository，选择URL标签页，输入你的小组仓库地址，选择本地路径，点击Clone。

#### 日常使用

- 拉取队友最新代码：在GitHub Desktop中，点击Fetch origin按钮。
- 修改代码后，可以在左侧看到有变动的文件，右侧看得到变更内容。左侧下方输入总结和可选的提交说明，点击Commit to main按钮。
- 提交后，点击Push origin按钮把改动推送到GitHub上。

[也可参考此CSDN教程](https://blog.csdn.net/qq_53123067/article/details/138466344)

### 如果遇到冲突了怎么办？（两个人改了同一个地方）

冲突就像两个人同时改同一个句子，Git无法自动合并，需要你商量一下留哪个版本就好。

#### 命令行

```bash
# 当你git pull时，如果出现冲突，Git会提示你哪些文件有冲突，并在文件中标记出冲突的部分。

git pull                 # 会提示冲突
# 用 VS Code 或记事本打开冲突文件，会看到类似：
# <<<<<<< HEAD
# 你的代码
# =======
# 队友的代码
# >>>>>>> branch
# 手动删掉 <<< === >>> 这些标记，保留最终需要的代码

git add .
git commit -m "（说明）"
git push
```

#### 图形界面工具

GitHub Desktop 会弹窗提示冲突，并且不会让你直接提交。

- 点击 Open in Visual Studio Code (或其他文本编辑器)
- 你会看到类似这样的标记（如果是在VSCode里看，会有绿色/蓝色背景，代表不同版本的代码）：
```plaintext
<<<<<<< HEAD
你写的代码
=======
队友写的代码
>>>>>>> main
```
- 手动删掉那些 <<< === >>> 标记，保留最终要的代码（比如把两段合并成一段）
- 保存文件，回到GitHub Desktop，输入提交说明，点击Commit to main，然后Push origin即可。

## 另

B306的GitHub组织也才去年成立没多久，所以其实挺空的。现在有的仓库就是```B306-CV-Examples```、```B306-MSPM0-ModuleLibs```、```B306-STM32-GeneralLibs```、```B306-STM32-ModuleLibs```这几个。

**抓  住  历  史  机  遇！**

组织需要你们的持续贡献才能变强！（）
