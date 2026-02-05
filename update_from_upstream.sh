#!/bin/bash
set -e

# switch to master and make sure we're up to date with GitHub
CURRENT_BRANCH=$(git branch --show-current)
git stash --all
git switch main
git fetch origin main
git pull origin main

# merge any updates from GitLab
git fetch upstream master
git merge upstream/master

# commit updates to GitHub
git add .
if ! git diff-index --quiet HEAD; then
  git commit -m "Merged in updates from upstream"
  git push origin main
fi

# restore original branch and working directory
git switch $CURRENT_BRANCH
git stash pop || true

echo "Done!"
