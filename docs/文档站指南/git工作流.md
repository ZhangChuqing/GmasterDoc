# Git 工作流

文档站仓库地址：[https://github.com/GMasterEmbeddedGroup/GmasterDoc](https://github.com/GMasterEmbeddedGroup/GmasterDoc)

我们采用 **Fork + Pull Request** 工作流，防止恶意修改，保证文档质量。

## 为什么用 PR？

- 所有修改需要经过审核才能合并到主分支
- 每次修改有清晰的历史记录，方便追溯
- 降低误操作和恶意修改的风险

## 完整流程

### 1. Fork 仓库

访问 [GmasterDoc 仓库](https://github.com/GMasterEmbeddedGroup/GmasterDoc)，点击右上角 **Fork** 按钮，将仓库复制到你的 GitHub 账号下。

### 2. Clone 到本地

```bash
git clone git@github.com:你的用户名/GmasterDoc.git
cd GmasterDoc
```

### 3. 添加上游仓库

```bash
git remote add upstream git@github.com:GMasterEmbeddedGroup/GmasterDoc.git
```

### 4. 创建功能分支

每次修改前，从最新的 `main` 分支创建新分支：

```bash
git checkout main
git pull upstream main        # 同步上游最新代码
git checkout -b add-my-doc    # 创建并切换到新分支（分支名建议描述修改内容）
```

### 5. 编写文档并提交

```bash
git add .
git commit -m "添加XX组焊接指南"
```

### 6. 推送到你的 Fork

```bash
git push origin add-my-doc
```

### 7. 发起 Pull Request

在 GitHub 上进入你的 Fork 仓库，点击 **Compare & pull request**，填写修改说明后提交。

### 8. 等待审核

维护者审核通过后，你的修改将合并到主仓库。

## 注意事项

- 每次修改前先 `pull upstream main` 同步，避免冲突
- 一个 PR 专注于一个主题，不要混入无关修改
- Commit message 简洁明了地描述做了什么
