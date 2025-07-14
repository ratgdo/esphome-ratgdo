#!/usr/bin/env python3
"""Update YAML files to use local paths instead of GitHub URLs for CI testing."""

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
    # Get the absolute path to the project root
    project_root = Path(__file__).parent.parent.absolute()

    # Get branch and fork info for dashboard_import
    branch, fork_repo = get_pr_info()

    print(f"Project root: {project_root}")
    print(f"Current branch: {branch}")
    if fork_repo and fork_repo != RATGDO_REPO:
        print(f"Fork repository: {fork_repo}")
    print("Updating YAML files to use local paths for CI testing...")

    # Process all YAML files
    for yaml_file in Path(".").glob("*.yaml"):
        with open(yaml_file, "r") as f:
            content = f.read()

        original = content

        # Update external_components to use local path
        if (
            "external_components:" in content
            and "type: git" in content
            and "ratgdo/esphome-ratgdo" in content
        ):
            # Replace the git source with local source, preserving indentation
            # This matches the exact structure: type: git, url: ..., ref: ...
            content = re.sub(
                r"type:\s*git\s*\n(\s+)url:\s*https://github\.com/ratgdo/esphome-ratgdo\s*\n\s+ref:\s*\w+",
                rf"type: local\n\1path: {project_root}/components",
                content,
            )

        # Update remote_package to use local path
        if "remote_package:" in content and "ratgdo/esphome-ratgdo" in content:
            # First update the URL
            content = re.sub(
                r"(remote_package:\s*\n\s*)url:\s*https://github\.com/ratgdo/esphome-ratgdo",
                rf"\1url: file://{project_root}",
                content,
            )
            # Then remove the ref line while preserving indentation
            # This matches the ref line and removes it completely
            content = re.sub(r"(\n\s+)ref:\s*\w+\n", r"\n", content)

        # Update dashboard_import to use the correct branch (not local file)
        if "dashboard_import:" in content:
            # Update @main to @branch
            content = re.sub(r"@main\b", f"@{branch}", content)

            # If this is a fork, update repository URLs
            if fork_repo and fork_repo != RATGDO_REPO:
                # Update dashboard imports to use fork
                content = re.sub(
                    rf"github://{RATGDO_REPO}/", f"github://{fork_repo}/", content
                )

        if content != original:
            with open(yaml_file, "w") as f:
                f.write(content)
            print(f"Updated {yaml_file}")

    print("Done!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
