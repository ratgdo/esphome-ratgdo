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
    # In GitHub Actions, use the workspace path
    if os.environ.get("GITHUB_ACTIONS") == "true":
        project_root = Path(os.environ.get("GITHUB_WORKSPACE", ".")).absolute()
    else:
        project_root = Path(__file__).parent.parent.absolute()

    # Get branch and fork info for dashboard_import
    branch, fork_repo = get_pr_info()

    print(f"Project root: {project_root}")
    print(f"Current branch: {branch}")
    if fork_repo and fork_repo != RATGDO_REPO:
        print(f"Fork repository: {fork_repo}")
    print("Updating YAML files to use local paths for CI testing...")

    # Process all YAML files in the project root
    for yaml_file in project_root.glob("*.yaml"):
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

        # Remove dashboard_import section completely FIRST
        if "dashboard_import:" in content and "package_import_url:" in content:
            # Remove the entire dashboard_import section including trailing newlines
            content = re.sub(
                r"dashboard_import:\s*\n\s+package_import_url:\s*github://[^\n]+\n+",
                "",
                content,
            )

        # Update remote_package to use local packages as a list
        if "remote_package:" in content and "ratgdo/esphome-ratgdo" in content:
            # Replace the entire remote_package with a list of local includes
            def replace_remote_package(match):
                files = match.group(1)
                # Convert [base.yaml] to just base.yaml
                files = files.strip("[]").strip()
                return f"packages:\n  - !include {project_root}/{files}\n"

            content = re.sub(
                r"packages:\s*\n\s+remote_package:\s*\n\s+url:\s*https://github\.com/ratgdo/esphome-ratgdo\s*\n(?:\s+ref:\s*\w+\s*\n)?\s+files:\s*\[([^\]]+)\]\s*\n(?:\s+refresh:\s*\S+\s*\n)?",
                replace_remote_package,
                content,
            )

        if content != original:
            with open(yaml_file, "w") as f:
                f.write(content)
            print(f"Updated {yaml_file}")

    print("Done!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
