#!/usr/bin/env python3
"""Update YAML files to use current Git branch instead of main."""

import json
import os
import re
import sys
from pathlib import Path

RATGDO_REPO = "ratgdo/esphome-ratgdo"


def get_current_branch():
    """Get current branch name from GitHub environment."""
    # For PRs, check GITHUB_REF first
    github_ref = os.environ.get("GITHUB_REF", "")
    if "/pull/" in github_ref:
        # It's a PR, get branch name from event file
        github_event_path = os.environ.get("GITHUB_EVENT_PATH")
        if github_event_path and os.path.exists(github_event_path):
            with open(github_event_path) as f:
                event_data = json.load(f)
                pr_data = event_data.get("pull_request", {})
                return pr_data.get("head", {}).get("ref", "main")

    # For pushes, extract branch from GITHUB_REF
    if github_ref.startswith("refs/heads/"):
        return github_ref.replace("refs/heads/", "")

    return "main"


branch = get_current_branch()

if branch == "main":
    print("On main branch, skipping ref updates")
    sys.exit(0)

print(f"Updating refs to: {branch}")

# Process all YAML files
for yaml_file in Path(".").glob("*.yaml"):
    with open(yaml_file, "r") as f:
        content = f.read()

    original = content

    # Update ref: main to ref: <branch>
    content = re.sub(r"(\s+)ref:\s*main\b", rf"\1ref: {branch}", content)

    # Update @main in dashboard imports
    content = re.sub(r"@main\b", f"@{branch}", content)

    # For remote_package sections without ref, add it after the URL
    # Look for patterns like:
    #   remote_package:
    #     url: https://github.com/ratgdo/esphome-ratgdo
    #     files: [...]
    pattern = (
        r"(remote_package:\s*\n\s*url:\s*https://github\.com/"
        + RATGDO_REPO.replace("/", r"\/")
        + r"\s*\n)(\s*files:)"
    )
    replacement = rf"\1\2ref: {branch}\n\3"

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
