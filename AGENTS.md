# Notes for LLM contributors

## Rule zero: prove it works before opening the PR

**Your job is to deliver code that is proven to work.** If you
have not proven the change works, it is not time to open the PR
yet. "It compiles", "ruff passes", and "the diff looks right"
are not proof. Proof is: the relevant tests run locally and
pass, the new behaviour is exercised by a test you added or
extended where one is possible, and any user-visible path you
touched has been executed end-to-end.

This is a firmware project that ships to real hardware
controlling real garage doors. The bar for "it works" is
correspondingly high:

- **Every LLM-authored PR must be tested on a real ratgdo
  board by the human operator before it leaves draft.** No
  exceptions. The agent itself almost certainly cannot do
  this; the operator running the agent must. See
  _Pull request rules → Open the PR as a draft_ below.
- Python codegen changes (under `components/ratgdo/`) must
  `esphome config <yaml>` cleanly for every affected board YAML,
  not just one. Schema mistakes only surface during config
  validation of a specific platform.
- C++ behavioural changes must be flashed to a real board and
  the affected path exercised end-to-end (open, close, stop,
  obstruction, learn, lock, whatever you touched) before the
  PR leaves draft. The opener protocol is timing-sensitive and
  "looks right" failure modes are common; a misbehaving change
  here can leave a user's garage door stuck open or refusing
  to close.
- If you the agent cannot flash hardware in your environment,
  say so explicitly in the PR body. State plainly that the
  change is unverified on hardware and is waiting on the
  operator to flash and exercise it. Do not imply coverage you
  did not actually achieve.

Opening a PR that turns out not to work wastes the reviewer's
time and is the single fastest way to lose trust on this repo.

The rest of this document covers how to dress up that proven
change for review. None of it matters if rule zero is not met.

---

Read this before opening a pull request against
`ratgdo/esphome-ratgdo`. This file is the short orientation for
agents; the user-facing README is [README.md](README.md).

## What this project is

`esphome-ratgdo` is the ESPHome external component for the
[ratgdo](https://ratcloud.llc) family of WiFi control boards
for residential garage door openers (Chamberlain / LiftMaster
Security+ 1.0 and 2.0, plus dry-contact for other brands). It
is consumed via ESPHome's `external_components:` mechanism — it
is not a standalone firmware. Users pick one of the board YAMLs
at the repo root, run it through ESPHome, and flash the
resulting binary.

The component itself is a mix of Python (ESPHome codegen and
config validation) and C++ (runtime, protocol state machines,
peripheral drivers). Both halves ship in the same PR for any
user-visible change.

Useful entry points:

| Path                                            | What                                                                                |
| ----------------------------------------------- | ----------------------------------------------------------------------------------- |
| `components/ratgdo/__init__.py`                 | top-level Python codegen, schema, observable subscriber counts                      |
| `components/ratgdo/ratgdo.{h,cpp}`              | `RATGDOComponent` — the main runtime class                                          |
| `components/ratgdo/secplus1.{h,cpp}`            | Security+ 1.0 protocol state machine                                                |
| `components/ratgdo/secplus2.{h,cpp}`            | Security+ 2.0 protocol state machine                                                |
| `components/ratgdo/dry_contact.{h,cpp}`         | dry-contact (non-Sec+) implementation                                               |
| `components/ratgdo/ratgdo_uart_esp32.{h,cpp}`   | ESP-IDF UART + RMT driver (replaces Arduino SoftwareSerial on ESP32)                |
| `components/ratgdo/ratgdo_uart_esp8266.h`       | ESP8266 SoftwareSerial path                                                         |
| `components/ratgdo/observable.{h,cpp}`          | minimal observer template used for sensor fan-out; sized at compile time            |
| `components/ratgdo/{binary_sensor,cover,light,lock,number,output,sensor,switch}/` | per-platform Python + C++ glue           |
| `base.yaml`, `base_secplusv1.yaml`, `base_drycontact.yaml` | shared YAML fragments included by board configs                          |
| `v*board*.yaml`, `v25iboard*.yaml`, `v32*.yaml` | per-board top-level configs; the build matrix in CI enumerates these                |
| `static/`                                       | Web Installer assets (HTML, images, per-board YAMLs mirrored for download)          |
| `scripts/update_refs_for_ci.py`                 | rewrites `external_components` refs to local paths during CI builds                 |
| `tests/`                                        | pytest suite (currently covers `scripts/`; expand it when adding new Python tools)  |
| `.github/workflows/build.yml`                   | runs pytest, then builds every board YAML via `ratgdo/esphome-build-action`         |

## Framework: ESP-IDF vs Arduino

Most ESP32 boards in this repo use the **ESP-IDF** framework.
The project previously depended on Arduino (for SoftwareSerial),
but [PR #577](https://github.com/ratgdo/esphome-ratgdo/pull/577)
replaced that with hardware UART + RMT and dropped the Arduino
layer. Do not reintroduce Arduino-only APIs on ESP32 paths.

Exceptions:

- The **v3.2 Disco** board uses Arduino because its VL53L4CX
  distance sensor library and `Wire` I2C library require it.
- All **ESP8266** boards use Arduino — ESPHome requires it on
  that platform.

If you are touching the UART, RMT, or interrupt paths, make
sure your change compiles cleanly on both `esp32` (IDF) and
`esp8266` (Arduino) YAMLs that use it.

## Compile-time observable sizing

`components/ratgdo/__init__.py` keeps subscriber counts
(`RATGDOData`) for each observable (door state, distance,
vehicle detection, etc.) and feeds them as template parameters
to the C++ side. This is so `observable<T, N>` can stack-allocate
its subscriber array at compile time and avoid heap churn on
small MCUs.

If you add a new platform that subscribes to one of these
observables, you **must** call the matching
`subscribe_<thing>()` helper from that platform's Python
codegen. If you add a new observable, you must add a new
counter field and a `subscribe_*` helper, and thread the
template parameter through to the C++ class. Skipping this
step manifests as silently-dropped callbacks at runtime, not
as a compile error — easy to miss in review.

## Pull request rules

These are the rules agents most often violate. Treat them as
mandatory.

### 1. Use the project's `## What / ## Why / ## How / ## Testing` PR body

Unlike some other repos, `esphome-ratgdo` *does* use this
layout. Match the recent landed PRs (see
[#620](https://github.com/ratgdo/esphome-ratgdo/pull/620),
[#598](https://github.com/ratgdo/esphome-ratgdo/pull/598),
[#594](https://github.com/ratgdo/esphome-ratgdo/pull/594) for
the right shape):

```markdown
## What

<one or two sentences, plain prose>

## Why

<the user-visible symptom, or the constraint that motivated the change;
link issues with "Closes #NNN" or "Fixes #NNN" here when applicable>

## How

<short description of the approach; bullet the files touched only if it
helps a reviewer orient, not as filler>

## Testing

<exactly what you ran and what passed; if you flashed hardware, say which
board and which protocol. If you could not flash, say so.>

Closes #NNN
```

Keep each section short. A couple of sentences per section is
plenty. Long multi-section essays with bolded sub-headings are
not the style here.

### 2. Open the PR as a draft, and leave it that way

Use `gh pr create --draft`. **Every LLM-authored submission
must be fully reviewed by a human AND tested on real ratgdo
hardware before it is marked ready out of draft, with no
exceptions.** This repo ships firmware to physical garage door
openers; "the code looks right" and "CI builds pass" are not
acceptable substitutes for flashing the change to a real board
and exercising the affected path. That review and that
hardware test are the responsibility of the person running the
agent, not of the project maintainers; do not shift either
burden onto them.

Do not mark the PR ready yourself, and do not request
reviewers from the agent session. The human who reviewed the
change, flashed it to a board, confirmed it behaves correctly,
and then flipped the PR out of draft is the one who routes
it. If the operator has not yet completed the hardware test,
the PR stays in draft, full stop.

### 3. Disclose the agent, do not advertise it

Disclosure is required, advertising is not welcome. Put one
plain line at the bottom of the PR body naming the agent that
drafted the change, for example:

```
Drafted with <agent name and version>; reviewed by <human handle>.
```

That single line is enough. Beyond that:

- **No `Co-Authored-By:` trailers** for an LLM or any AI tool,
  in commits or in the PR body. Attribution goes to the human
  who reviewed the change.
- **Agent output goes in a footer below the PR summary, ideally
  in a collapsed `<details>` block.** The
  What / Why / How / Testing sections come first and read like
  a human wrote them. Anything the agent wants to surface for
  reviewers (scan results, lint output, branch hygiene notes)
  goes underneath that:

  ```markdown
  <details>
  <summary>Agent run details (optional, for reviewers)</summary>

  Lint: <command and result>
  Pytest: <command and result>
  </details>
  ```

  What is not OK is mixing this content into the template
  sections themselves, or pushing it above the human-readable
  summary so reviewers have to scroll past it.
- No `🤖`, `✨`, `🚀` emoji decoration in commit messages, PR
  titles, or PR bodies. Project style is plain prose.
- Commit messages and PR prose should read as if a human
  contributor wrote them. Specifically:
  - **No em-dashes (`—`)** and no dashes used as sentence
    separators (`foo - bar`). Use a semicolon or a comma. This
    is the strongest tell for AI-generated prose; reviewers do
    read for it.
  - No "Let me", "I'll", or first-person narration of what the
    agent did. Describe the change, not the author.
  - No filler sections ("Overview", "Summary of changes", "Key
    takeaways") on top of the template.

### 4. Commit hygiene

- One logical change per PR. If a refactor and a bugfix are
  bundled together, split them.
- Pre-commit auto-fixes (`ruff`, `ruff format`, `clang-format`
  Webkit, trailing-whitespace, end-of-file-fixer) run on commit
  and rewrite files in place; when a hook rewrites a file the
  commit aborts, so re-stage and commit again.
- The repo does **not** use Conventional Commits as a CI gate.
  Recent landed subjects are short imperative or descriptive
  prose (e.g. `Suppress strapping pin warning on v32 LED
  (GPIO2)`, `Wake main loop from UART RX ISR on ESP32`, `Fix
  signed/unsigned type mismatch in obstruction_loop()`). Match
  that style; do not force `feat:` / `fix:` prefixes onto every
  commit. `chore(ci):` is fine for the dependabot-style CI
  bumps because that is what dependabot emits.

## Tests

Install the Python test deps and run the suite:

```bash
pip install -r requirements-test.txt
pytest tests/ -v
```

The pytest suite currently covers the helper scripts under
`scripts/`. If you add a new Python helper, add tests for it
under `tests/` and wire them in — CI runs `pytest tests/ -v`
and the firmware build matrix is gated on it passing.

For ESPHome config validation, run:

```bash
esphome config <board>.yaml
```

against every board YAML your change can affect. The CI
matrix in `.github/workflows/build.yml` enumerates all the
shipped boards; if you broke a YAML you did not test, CI will
catch it, but you will have wasted a build cycle.

## Pre-commit and formatting

The repo uses `pre-commit` (see `.pre-commit-config.yaml`):

- **Python**: `ruff` with `--fix`, then `ruff-format`. Config
  in `pyproject.toml` targets py311 and has an opinionated
  rule set; do not relax it.
- **C++**: `clang-format -style=Webkit -i`. Do not hand-format
  C++ — let clang-format do it.
- Plus `trailing-whitespace`, `end-of-file-fixer`,
  `check-added-large-files`.

Install once with `pre-commit install` so the hooks run on
every commit. The `[pre-commit.ci]` bot PRs you see in the log
are automatic autoupdates; do not try to do that work by hand.

## Web Installer and `static/`

The contents of `static/` are served as the
[Web Installer](https://ratgdo.github.io/esphome-ratgdo/) via
GitHub Pages from the `consolidate` + `deploy` jobs in CI. The
per-board YAMLs in `static/` mirror the top-level YAMLs so end
users can download them; if you add or rename a top-level YAML,
update the `static/` mirror and the matrix in
`.github/workflows/build.yml` in the same PR.

## Things not to do

- Do not open a PR for code you have not proven works (see
  _Rule zero_ at the top of this file). For C++ runtime
  changes, "proven" usually means flashed to a real board; if
  you cannot do that, say so honestly in the PR body.
- Do not reintroduce Arduino-only APIs on ESP32 paths that
  have been moved to ESP-IDF. The v3.2 Disco and ESP8266
  builds are the only places Arduino still belongs.
- Do not add a new observable subscription from a platform
  without bumping the corresponding `RATGDOData` counter in
  `components/ratgdo/__init__.py`; the C++ side relies on it
  for compile-time sizing.
- Do not add a new top-level YAML without also adding it to
  the `firmware` matrix in `.github/workflows/build.yml` and
  mirroring it under `static/`.
- Do not commit build artefacts: `__pycache__/`, `.pio/`,
  `.esphome/`, compiled `*.bin`, or anything ESPHome generates
  into `.esphome/build/`.
- Do not commit local-only files like `smallgarage.yaml`,
  device log dumps, or generated reports.
- Do not add `Co-Authored-By` trailers for LLM tools, in
  either commits or the PR body.
- Do not mix agent-generated scan output, test summaries, or
  pipeline reports into the template sections. Put them in a
  collapsed `<details>` footer below the PR summary instead.
- Do not use em-dashes or sentence-separating dashes in PR
  prose or commit messages.
- Do not mark the PR ready for review yourself; that is the
  call of the human running the agent, not the agent itself.
- Do not request reviewers from the agent session; the human
  who flips the PR out of draft will route it.
