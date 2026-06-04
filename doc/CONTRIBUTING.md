# How to Contribute to Triple Helix Robot Code

This guide walks through everything you need to get set up and start contributing code — from installing the development environment, to writing and testing changes, to getting your work reviewed and merged.

---

## 1. Development Environment Setup

### WPILib

WPILib is the core framework for FRC robot programming. Install [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) by following the official setup guide. The installer bundles everything you need in one place:

- Java 17 JDK
- VS Code with the WPILib extension pre-configured
- Gradle build tooling
- Simulation libraries and tools

Follow the installer prompts all the way through. When it finishes, always open the **WPILib VS Code** for robot development — do not use a separately installed copy of VS Code, as it will be missing the WPILib extension and Java configuration.

- **Windows:** WPILib VS Code is installed to `C:/Users/Public/wpilib/2026/vscode`. The installer places a shortcut on the desktop.
- **Mac:** WPILib VS Code is installed to `~/wpilib/2026/vscode`. You can open it from Finder or pin it to your Dock for easy access.

### Git

Git is the version control system we use to manage code changes, collaborate, and keep a history of the project.

**Windows:** Download and install the [latest version of Git for Windows](https://git-scm.com/download/win). During the installer, when you reach the "Additional components" step, do **not** check the Git Credential Manager option — leave it unchecked. All other defaults are fine.

**Mac:** Git comes pre-installed as part of the Xcode Command Line Tools. Open Terminal and run:

```bash
git --version
```

If Git is not installed, macOS will prompt you to install the Command Line Tools — follow the on-screen instructions. Alternatively, you can install a newer version via [Homebrew](https://brew.sh):

```bash
brew install git
```

### Set up your repositories folder

Before cloning, make sure the folder where you will store repositories exists. Triple Helix uses a standard location so that everyone on the team knows where to find code:

| Platform | Path |
|----------|------|
| Windows  | `C:/Users/<your-username>/repositories` |
| Mac      | `~/repositories` (i.e. `/Users/your-username/repositories`) |

**Windows:** Open File Explorer and navigate to `C:/Users/<your-username>` (your home directory). If the `repositories` folder is not there, create it.

**Mac:** Open Terminal and run:

```bash
mkdir -p ~/repositories
```

### Clone the repository

With the folder ready, clone the repo into it using VS Code's built-in Git integration:

1. Open WPILib VS Code.
2. Press `Ctrl+Shift+P` (Windows) or `Cmd+Shift+P` (Mac) to open the command palette.
3. Type **Git: Clone** and select it. A box will appear asking for the repository URL.
4. In a browser, log into your GitHub account and navigate to the Triple Helix organization. Open the repository you want to work on and copy the HTTPS clone URL from the **Code** button.
5. Paste the URL into the VS Code box and press Enter.
6. When prompted for a location, navigate to your repositories folder (`C:/Users/<your-username>/repositories` on Windows, `~/repositories` on Mac) and confirm.

VS Code will clone the repository and ask if you want to open it — click **Open**.

> **Important:** Before you begin any work, make sure your local `main` branch is in sync with the remote on GitHub. You can check this by comparing the commit hash shown in the bottom-left status bar of VS Code against the latest commit shown on the GitHub repository page. If they are not the same, do not start working — talk to a mentor to get your local copy synced up first.

---

## 2. Working in Local Branches

The `main` branch is the shared, stable version of the code. Your local copy of `main` should always be a direct mirror of the remote `main` on GitHub — **never make changes directly on `main`**. Instead, all development happens in a personal feature branch, which is your own copy of the code where you can make changes freely without affecting anyone else.

### Create a branch

Switch to `main` first and make sure it is up to date:

1. In the bottom-left corner of VS Code, click the branch name and select `main`.
2. Pull to make sure it is current: `Ctrl+Shift+P` → **Git: Pull**.

Then create your feature branch:

```
Ctrl+Shift+P → Git: Create Branch
```

Give it a short, descriptive name that reflects what you are working on, for example:
- `fix-intake-speed`
- `auto-path-center-note`
- `tune-drive-pid`

The current branch name is always shown in the lower-left corner of VS Code. Make sure you see your new branch name there before making any changes.

### Make and test your changes

1. Make your code changes in the branch. Keep changes focused — try not to mix unrelated fixes in the same branch.
2. Build to compile and check formatting. The command depends on your platform and terminal:
   - **Mac / Linux / Windows Git Bash or PowerShell:** `./gradlew build`
   - **Windows Command Prompt:** `gradlew build`
3. Test your changes on the robot or in simulation. Make sure the robot behaves as expected in the areas you modified.

### Commit and push your branch daily

**Push your branch to GitHub at the end of every shop session.** This is non-negotiable. Laptops die, get lost, get borrowed, or need to be used by someone else for debugging. Having your work pushed to GitHub means it is never truly lost and others can help you if needed.

To commit your work:

1. Open the Source Control panel in VS Code (the branch icon on the left sidebar).
2. Review the list of changed files. Click the **+** next to each file you want to include, or use the `...` menu at the top and choose **Stage All Changes**.
3. Type a commit message in the text field at the top of the panel. Write something that clearly describes what you changed — this becomes part of the permanent history of the project (see [Code Standards](#4-code-standards) below for tips).
4. Click the checkmark (✓) to commit.

Then push to GitHub:

```
Ctrl+Shift+P → Git: Push
```

Your branch will appear on GitHub under your branch name. You can verify by visiting the repository on GitHub and checking the branch dropdown.

---

## 3. Submitting Changes

When your feature is complete and tested, it is time to get it into `main`. This involves syncing with any changes that happened in `main` while you were working, resolving any conflicts, and opening a pull request for review.

### Sync with main

Other people may have merged changes into `main` while you were working on your branch. Before opening a pull request, you need to bring those changes into your branch so that the history is clean and there are no surprises during review.

1. Fetch the latest state of the remote repository. This updates your local knowledge of what is on GitHub without changing any of your files:
   ```
   Ctrl+Shift+P → Git: Fetch
   ```

2. Make sure you are on your feature branch (check the lower-left corner).

3. Rebase your branch onto the updated `main`. Rebasing replays your commits on top of the latest `main`, producing a clean linear history:
   ```
   Ctrl+Shift+P → Git: Rebase Branch...
   ```
   Choose `main` (or `origin/main`) as the branch to rebase onto.

   If you are not comfortable with rebase, you can merge instead:
   ```
   Ctrl+Shift+P → Git: Merge Branch...
   ```
   Choose `main`. This creates a merge commit but is safer if you are unfamiliar with rebase.

### Resolve conflicts

If `main` has changes that overlap with your branch, Git will flag conflicts. Do not panic — conflicts are normal and VS Code makes them straightforward to resolve.

1. After the rebase or merge, any files with unresolved conflicts will appear in the Source Control panel with a double-headed arrow icon.
2. Open each conflicted file. VS Code will show both versions inline — your changes and the incoming changes from `main`.
3. Right-click the file in the Source Control panel and choose **Open Merge Editor** for a side-by-side view, or resolve directly in the editor using the **Accept Current**, **Accept Incoming**, or **Accept Both** buttons that appear inline.
4. Use judgment when resolving: if `main` changed a section for a good reason (e.g. a refactor or bug fix), accept that version and re-apply your intent on top of it. If both changes are independent and additive, accept both.
5. After resolving each file, save it. Then stage it by clicking **+** next to the file in the Source Control panel. Git does not automatically detect that you have resolved a conflict — you must stage it explicitly.
6. Once all conflicts are staged, build and test the affected areas to make sure the combined result behaves correctly.
7. Commit the resolved state with a message like `Resolve merge conflicts with main`.

### Final check before opening a PR

Run the full build one more time before pushing. Use `./gradlew` on Mac/Linux or Git Bash; use `gradlew` in Windows Command Prompt:

```bash
./gradlew build
```

Formatting is applied automatically on every commit by the pre-commit hook (installed on first build), so it should already be clean. CI will run `spotlessCheck` on your PR and will fail if formatting is not clean — if that happens, run `./gradlew spotlessApply`, commit, and push.

### Open a pull request

1. Push your branch to GitHub one final time to make sure the latest is there:
   ```
   Ctrl+Shift+P → Git: Push
   ```
2. Go to the repository on GitHub. You should see a banner suggesting you open a pull request for your recently pushed branch — click it. If not, navigate to the **Pull requests** tab and click **New pull request**, then select your branch.
3. Set the base branch to `main`.
4. Write a clear PR title and description:
   - What does this change do?
   - Why was it needed?
   - Is there anything reviewers should pay particular attention to, or anything that still needs follow-up?
5. Keep PRs focused — one feature or bug fix per PR. A focused PR is much easier to review and much less likely to introduce unexpected problems.

### Review and merge

PRs must be approved by **a software mentor or 2 other senior programmers** before they can be merged. CI will automatically run `./gradlew build` and `spotlessCheck` on every push to your PR — both must pass before merging. If CI fails, check the error output in the Actions tab on GitHub and fix the issue before requesting review.

Once approved and CI is green, a mentor or senior programmer will merge the PR.

---

## 4. Code Standards

### Formatting

This project uses [Spotless](https://github.com/diffplug/spotless) with Google Java Format to keep code style consistent across the whole codebase. Formatting is applied automatically on every build and on every commit via a pre-commit hook in `.githooks/`. The hook is installed automatically the first time you run `./gradlew build`, so in most cases you do not need to think about it. If you ever want to apply it manually:

```bash
./gradlew spotlessApply
```

### Commit messages

Commit messages become the permanent history of the project and are read by your teammates and mentors. Write them to be useful:

- Use present tense and imperative mood: `Fix intake roller speed` not `Fixed intake roller speed` or `Fixes intake roller speed`
- Be specific: `Tune drive PID for carpet friction` is better than `Update constants`
- Keep the first line short (under 72 characters). If more explanation is needed, add a blank line and a longer description below

Examples of good commit messages:
- `Fix intake roller speed constant`
- `Add autonomous path for center note`
- `Resolve merge conflicts with main`
- `Apply Spotless formatting`

### CI

The build badge at the top of the README reflects the current state of CI on `main`. Do not merge a PR that would leave CI failing. If CI is failing on your PR, it is your responsibility to investigate and fix it before asking for review.
