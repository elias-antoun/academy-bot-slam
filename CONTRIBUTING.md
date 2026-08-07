# Contributing to AcadBot

Every homework in the SLAM track has a **code deliverable**, and every code
deliverable is handed in the same way: as a **pull request** from your own fork.
This page is the workflow. Read it once, in week 1; after that just follow the
checklist at the bottom.

The point is not the feature. Every change you are asked to make is small enough
to write in under an hour. The point is that you can take a change through the
loop a professional team actually uses — branch, commit, push, PR, review,
revise, merge — without anyone walking you through it.

---

## One-time setup

**1. Fork the repo.** On GitHub, open
`WaelSayegh/academy-bot-slam` and press **Fork**. You now have
`<your-username>/academy-bot-slam`.

**2. Clone your fork** — into `code/`, replacing the copy you already have if
you cloned it directly:

```bash
cd code
git clone git@github.com:<your-username>/academy-bot-slam.git
cd academy-bot-slam
```

**3. Add the original as `upstream`.** Your fork will fall behind as fixes land;
this is how you catch up.

```bash
git remote add upstream git@github.com:WaelSayegh/academy-bot-slam.git
git remote -v          # origin = yours, upstream = the class repo
```

**4. Tell git who you are**, if you never have:

```bash
git config --global user.name  "Your Name"
git config --global user.email "you@example.com"
```

> **`gh` is optional.** The GitHub CLI makes step 5 below a one-liner, but every
> instruction here also works from the GitHub website. It is not in the
> container image — install it on your host if you want it.

---

## The loop, every week

### 1. Start from an up-to-date `main`

Never work on `main`, and never start a new week's work on top of last week's
branch. Both make your PR unreviewable.

```bash
git checkout main
git pull upstream main        # fetch the class repo's latest
git push origin main          # keep your fork's main in sync too
```

### 2. Branch

One branch per deliverable, named for the session and the change:

```bash
git checkout -b s1/stop-after-n-laps
```

Use `s1/`, `s2/`, `s3/`, `s4/` as the prefix, then a short kebab-case
description of *what changes*, not what session it is for.

Good: `s2/forward-slam-params-file` · `s3/parameterise-camera-intrinsics`
Bad: `s2/homework` · `my-branch` · `fix`

### 3. Build and test before you commit

A PR that does not build is not a submission. Inside the container:

```bash
cd /ros2_ws
colcon build --symlink-install && source install/setup.bash
```

Then actually run the thing you changed. Every homework tells you what the
observable result should be — get that result before you commit, not after your
PR is reviewed.

### 4. Commit

Small commits, each one a complete thought. The message is written for the
person reading it in six months:

```
Stop square_driver after a configurable number of laps

The node drove forever, so students had to guess when the second lap
finished and Ctrl-C at the right moment. A `laps` parameter (0 = the old
forever behaviour) makes the two-lap measurement repeatable, and the node
now publishes a zero Twist before shutting down so the robot actually
stops instead of coasting on the last command.
```

The rules that matter:

- **Subject line under ~72 characters, imperative mood** — "Add", "Fix",
  "Stop", not "Added" or "Adding".
- **Blank line, then a body that says *why*.** The diff already shows what
  changed. It cannot show what was wrong before.
- **No `WIP`, `asdf`, `final`, `final2`.** Squash them before you push:
  `git rebase -i upstream/main`.
- **Never commit build output.** `build/`, `install/` and `log/` are
  gitignored. If `git status` shows them, stop and fix the ignore before you
  add anything.

### 5. Push and open the PR

```bash
git push origin s1/stop-after-n-laps
```

GitHub prints a URL that opens the PR form. Or, with `gh`:

```bash
gh pr create --repo WaelSayegh/academy-bot-slam \
  --base main --title "Stop square_driver after a configurable number of laps"
```

Check the PR header before you submit: it must read
**`WaelSayegh/academy-bot-slam : main  ←  <you>/academy-bot-slam : s1/...`**.
Opening it against your own `main` is the single most common mistake, and it
means nobody sees your work.

### 6. Write the description

This is graded. A PR nobody can review is not finished work. Use this shape:

```markdown
## What
One or two sentences. What does this change do?

## Why
What was wrong, annoying, or missing before.

## How to verify
The exact commands a reviewer runs, and what they should see.

    ros2 launch acadbot_control square_driver.launch.py
    # expect: two "Completed a full loop" lines, then the node exits
    # and the robot is stationary

## Evidence
A screenshot, a log excerpt, or the numbers you measured.
```

### 7. Respond to review

You will get comments. That is the normal outcome of a PR, not a bad grade.

- Answer every comment, even if the answer is "good catch, fixed in `a1b2c3d`".
- Push fixes as **new commits** on the same branch — do not close the PR and
  open a new one. The PR updates itself.
- Disagree if you think you are right, and say why. "Because the reviewer said
  so" is a worse reason to change code than any technical argument.
- Mark a thread resolved only once you have actually addressed it.

### 8. Review someone else's

From Session 2 onward you will be assigned **one classmate's PR to review**.
Leave at least three comments. Aim for these, in order of usefulness:

1. Something that is wrong, or will break on someone else's machine.
2. Something you did not understand and had to read twice.
3. Something you liked and would copy.

"LGTM" is not a review. Neither is a list of typos.

---

## Checklist before you press "Create pull request"

- [ ] Branched off an up-to-date `upstream/main`, not off last week's branch.
- [ ] `colcon build --symlink-install` succeeds with no new warnings.
- [ ] You ran the thing and saw the result the homework asked for.
- [ ] `git status` is clean — no `build/`, `install/`, `log/`, no stray maps.
- [ ] `git diff upstream/main` contains **only** the change you meant to make.
- [ ] Commit subjects are imperative and under ~72 characters.
- [ ] The PR targets `WaelSayegh/academy-bot-slam : main`.
- [ ] The description has What / Why / How to verify / Evidence.

---

## When it goes wrong

**I committed to `main` by mistake.**
```bash
git branch s1/my-work          # save the work on a new branch
git reset --hard upstream/main # put main back
git checkout s1/my-work
```

**My PR shows hundreds of changed files.** You branched off the wrong thing, or
you committed `build/`. Check `git diff upstream/main --stat`. Rebase onto a
fresh `upstream/main`:
```bash
git fetch upstream && git rebase upstream/main
```

**I have a merge conflict.** Git marks the clashing region in the file with
`<<<<<<<`, `=======` and `>>>>>>>`. Open it, decide what the file should
actually say, delete all three markers, then `git add <file>` and
`git rebase --continue`. Do not accept both sides blindly.

**I need to change my last commit message.** `git commit --amend`, then
`git push --force-with-lease origin <branch>` if you already pushed. Use
`--force-with-lease`, never plain `--force`.

**I have no idea what state I am in.** `git status` and `git log --oneline -10`
answer that ninety percent of the time. Ask before you delete anything.
