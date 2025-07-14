#!/usr/bin/env python3
"""Update YAML files to use current Git branch instead of main."""

import json
import os
import re
import sys
from pathlib import Path
from typing import Optional, Tuple

RATGDO_REPO = "ratgdo/esphome-ratgdo"


def get_pr_info() -> Tuple[str, Optional[str]]:
    """Get PR branch and repository info from GitHub environment."""
    # For PRs, check GITHUB_REF first
    github_ref = os.environ.get("GITHUB_REF", "")
    if "/pull/" in github_ref:
        # It's a PR, get info from event file
        github_event_path = os.environ.get("GITHUB_EVENT_PATH")
        if github_event_path and os.path.exists(github_event_path):
            with open(github_event_path) as f:
                event_data = json.load(f)
                pr_data = event_data.get("pull_request", {})
                head = pr_data.get("head", {})
                branch = head.get("ref", "main")
                repo = head.get("repo", {})
                fork_repo = repo.get("full_name")  # e.g., "someuser/esphome-ratgdo"
                return branch, fork_repo

    # For pushes, extract branch from GITHUB_REF
    if github_ref.startswith("refs/heads/"):
        branch = github_ref.replace("refs/heads/", "")
        return branch, None

    return "main", None


def main():
    """Main function."""
    branch, fork_repo = get_pr_info()

    if branch == "main":
        print("On main branch, skipping ref updates")
        return 0

    print(f"Updating refs to: {branch}")
    if fork_repo and fork_repo != RATGDO_REPO:
        print(f"Using fork repository: {fork_repo}")

    # Process all YAML files
    for yaml_file in Path(".").glob("*.yaml"):
        with open(yaml_file, "r") as f:
            content = f.read()

        original = content

        # Only process files that reference the ratgdo repository
        if RATGDO_REPO not in content and "ratgdo/esphome-ratgdo" not in content:
            continue

        # Update ref: main to ref: <branch>
        content = re.sub(r"(\s+)ref:\s*main\b", rf"\1ref: {branch}", content)

        # Update @main in dashboard imports
        content = re.sub(r"@main\b", f"@{branch}", content)

        # If this is a fork, update repository URLs
        if fork_repo and fork_repo != RATGDO_REPO:
            # Update repository URL in external_components
            content = re.sub(
                rf"(url:\s*https://github\.com/){RATGDO_REPO}",
                rf"\1{fork_repo}",
                content,
            )
            # Update dashboard imports to use fork
            content = re.sub(
                rf"github://{RATGDO_REPO}/", f"github://{fork_repo}/", content
            )

        # For remote_package sections without ref, add it after the URL
        # Look for patterns like:
        #   remote_package:
        #     url: https://github.com/<repo>/esphome-ratgdo
        #     files: [...]
        repo_pattern = fork_repo if fork_repo else RATGDO_REPO
        pattern = (
            r"(remote_package:\s*\n\s*url:\s*https://github\.com/"
            + repo_pattern.replace("/", r"\/")
            + r"\s*\n)(\s*files:)"
        )

        # Check if this pattern exists without a ref line
        if re.search(pattern, content) and not re.search(
            pattern.replace(r"(\s*files:)", r"(\s*ref:.*\n\s*files:)"), content
        ):
            # Get the indentation from the url line
            match = re.search(r"(remote_package:\s*\n(\s*)url:)", content)
            if match:
                indent = match.group(2)
                content = re.sub(pattern, rf"\1{indent}ref: {branch}\n\2", content)

        if content != original:
            with open(yaml_file, "w") as f:
                f.write(content)
            print(f"Updated {yaml_file}")

    print("Done!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
