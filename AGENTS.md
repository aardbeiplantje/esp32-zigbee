# Agent instructions

## Git workflow

- Do not create, switch to or work on branches unless the user explicitly asks for one. Work and commit on the branch that is checked out, which is `main` by default.
- When the user asks to finish or ship work, do all of these:
  1. Commit the changes.
  2. Merge every other local and remote branch into `main`.
  3. Delete the branches that were merged, locally and on the remote.
  4. Push `main`.
- Commit messages describe the change in plain words. Do not add model names or agent attribution lines unless the user asks for them.
