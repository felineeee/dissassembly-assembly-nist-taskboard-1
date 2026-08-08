### 🛠 GIT CHEATSHEET (GitHub Operations)

**Configuration & Setup**

```bash
git config --global user.name "Your Name"
git config --global user.email "youremail@example.com"
git config --global core.fileMode false
git init
git remote add origin <url>

```

**Daily Workflow**

```bash
git pull origin main
git checkout -b <branch_name>
git status
git add .
git commit -m "feat: <description>"
git push origin <branch_name>

```

**Branch Management**

```bash
git branch -a
git checkout <branch_name>
git checkout main
git merge <branch_name>
git branch -d <branch_name>

```

**Undoing & Reverting**

```bash
git checkout -- <file_path>
git reset --soft HEAD~1
git reset --hard origin/main
git clean -fd

```

**Logs & Comparison**

```bash
git log --oneline --graph --all
git diff <file_path>
git remote -v

```

**Stashing**

```bash
git stash
git stash pop
git stash list

```

---

Would you like me to generate a specific `.gitignore` file for your ROS 2
project to keep these commands clean?
