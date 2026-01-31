# SSH使用指南

## 生成SSH
本指南以 **GitHub** 为例，演示如何生成并配置 SSH 密钥，用于安全访问远程仓库。

---

### 1. 生成密钥（推荐使用 ed25519）
```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```
按提示操作，一般直接回车使用默认路径。密码可选，但**建议**设置以提升安全性。输入密码时**不会显示字符**，属正常现象

### 2. 启动 SSH 代理
```bash
eval "$(ssh-agent -s)"
```

### 3. 添加私钥到代理
```bash
ssh-add ~/.ssh/id_ed25519
```
### 4. 查看公钥内容（复制到剪贴板）
```bash
cat ~/.ssh/id_ed25519.pub
```

### 5. 将公钥添加到GitHub/其他平台
1. 登录你的GitHub账户。
2. 进入“Settings” > “SSH and GPG keys” > “New SSH key”。
3. 粘贴你的公钥内容，保存。


### 6. 将远程地址从HTTPS修改为SSH
要将Git仓库的远程URL从HTTPS修改为SSH，可以按照以下步骤操作：
#### 6.1 **查看当前远程URL**  
   使用以下命令查看当前的远程URL：
   ```bash
   git remote -v
   ```
#### 6.2 **修改远程URL**  
   使用以下命令将远程URL从HTTPS修改为SSH：
   ```bash
    git remote set-url origin git@github.com:username/repository.git
   ```
 请将`username`和`repository`替换为你的GitHub用户名和仓库名称。
#### 6.3 **验证修改**  
   再次使用以下命令查看远程URL，确保修改成功：
   ```bash
   git remote -v
   ```
### 7. 测试SSH连接
```bash
ssh -T git@github.com
```
如果看到类似以下的消息，说明SSH配置成功：
```
Hi username! You've successfully authenticated, but GitHub does not provide shell access.
```
---