#!/usr/bin/env python3

'''
Check PR branch commit conventions.

Validates that the commits a pull request adds on top of its base branch:
  - contain no merge commits
  - contain no fixup!/squash!/amend! commits
  - are not GitHub web-interface commits ("Apply suggestions from code
    review", "Update <file>", ...) which should have been squashed away
  - are not flagged WIP/DEBUG/TMP by their own subject line
  - have subject lines <= 160 characters
  - do not use a placeholder author email
  - do not lose the trailing newline on a changed source file

Brazenly adapted from ArduPilot's Tools/scripts/check_branch_conventions.py.

Run locally with, e.g.:
  .github/scripts/check_branch_conventions.py --base-branch origin/master
'''

from __future__ import annotations

import argparse
import os
import pathlib
import re
import subprocess
import sys

DOCS_URL = "https://ardupilot.org/dev/docs/submitting-patches-back-to-master.html"
MAX_SUBJECT_LEN = 160

# Subjects whose leading token marks the commit as unfinished, in any of the
# spellings people use: "WIP", "WIP:", "[WIP]", "(wip)".  A bare word without
# a delimiter is deliberately not matched, so that prose such as
# "TEMP sensor: ..." or "Debug the foo handler" is left alone.
WIP_MARKER_RE = re.compile(
    r'^\s*(?:\[\s*(?P<bracketed>[A-Za-z]+)\s*\]'
    r'|\(\s*(?P<parenthesised>[A-Za-z]+)\s*\)'
    r'|(?P<bare>[A-Za-z]+)\s*:)',
)
WIP_MARKERS = {
    "DEBUG",
    "DRAFT",
    "FIXME",
    "TEMP",
    "TMP",
    "WIP",
}

# Subjects GitHub's web interface generates for you.  These reach master when
# a suggestion is committed from the review UI and the branch is merged
# without being tidied up first; the change belongs squashed into the commit
# it is fixing.
GITHUB_UI_SUBJECT_RES = [
    re.compile(r'^Apply (batched )?suggestions from code review$'),
    re.compile(r'^Update \S+$'),
    re.compile(r'^Create \S+$'),
    re.compile(r'^Delete \S+$'),
    re.compile(r'^Rename \S+ to \S+$'),
    re.compile(r'^Add files via upload$'),
    re.compile(r'^Initial commit$'),
    re.compile(r'^Set theme jekyll-theme-\S+$'),
]

# extensions checked for a lost trailing newline
SOURCE_EXTENSIONS = {
    ".c", ".cc", ".cpp", ".cxx",
    ".h", ".hh", ".hpp",
    ".py",
    ".lua",
    ".js", ".ts",
    ".xml",
}

# Enable colour when attached to a terminal or running under GitHub Actions
_colour = sys.stdout.isatty() or os.environ.get('GITHUB_ACTIONS') == 'true'
_GREEN = '\033[32m' if _colour else ''
_RED = '\033[31m' if _colour else ''
_YELLOW = '\033[33m' if _colour else ''
_RESET = '\033[0m' if _colour else ''

PASS = f"{_GREEN}✓{_RESET}"
FAIL = f"{_RED}✗{_RESET}"
SKIP = f"{_YELLOW}~{_RESET}"

# subject prefixes 'git commit --fixup/--squash' write for 'rebase --autosquash'
AUTOSQUASH_PREFIXES = ("fixup!", "squash!", "amend!")

# git modes of regular files; symlinks (120000) and submodules are not checked
REGULAR_FILE_MODES = {"100644", "100755"}

AMEND_HINT = ("       Use 'git commit --amend' for the most recent commit, "
              "'git rebase -i' for an older one.")


def is_github_ui_subject(subject: str) -> bool:
    '''return True if subject is one GitHub's web interface wrote for you'''
    return any(regex.match(subject) for regex in GITHUB_UI_SUBJECT_RES)


class CheckBranchConventions():

    DEFAULT_UPSTREAM = "origin/master"

    def __init__(self, base_branch: str | None = None) -> None:
        self.base_branch = base_branch

    def run_git(self, args: list, check: bool = True) -> str:
        result = subprocess.run(
            ["git"] + list(args),
            capture_output=True, text=True,
        )
        if check and result.returncode != 0:
            raise RuntimeError(
                f"git {' '.join(args)} failed: {result.stderr.strip()}")
        return result.stdout

    def commit_subjects(self) -> list:
        '''return [(sha, subject)] for each commit the branch adds'''
        raw = self.run_git(
            ["log", f"{self.base_branch}..HEAD", "--reverse",
             "--pretty=format:%H %s"],
        ).strip()
        out = []
        for line in raw.splitlines():
            line = line.strip()
            if not line:
                continue
            sha, _, subject = line.partition(" ")
            out.append((sha, subject))
        return out

    def check_merge_commits(self) -> bool:
        merge_commits = self.run_git(
            ["log", f"{self.base_branch}..HEAD", "--merges", "--oneline"],
        ).strip()
        if merge_commits:
            print(f"{FAIL} Merge commits are not allowed:")
            for line in merge_commits.splitlines():
                print(f"         {line}")
            print("       Rebase your branch onto the base branch instead of "
                  "merging it in.")
            print(f"       See: {DOCS_URL}")
            return False
        print(f"{PASS} No merge commits.")
        return True

    def check_fixup_commits(self, commits: list) -> bool:
        bad = [(sha, subject) for sha, subject in commits
               if subject.startswith(AUTOSQUASH_PREFIXES)]
        if bad:
            print(f"{FAIL} fixup!/squash!/amend! commits are not allowed:")
            for sha, subject in bad:
                print(f"         {sha[:12]} {subject}")
            print("       Squash them with 'git rebase -i --autosquash "
                  f"{self.base_branch}'.")
            print(f"       See: {DOCS_URL}")
            return False
        print(f"{PASS} No fixup!/squash!/amend! commits.")
        return True

    def check_github_ui_commits(self, commits: list) -> bool:
        '''reject the commit subjects GitHub's web interface writes for you.

        These are the "Apply batched suggestions from code review" commits
        that appear when review suggestions are committed from the browser.
        The change itself is usually fine; it just needs squashing into the
        commit it fixes rather than landing on master as its own commit.
        '''
        bad = [(sha, subject) for sha, subject in commits
               if is_github_ui_subject(subject)]
        if bad:
            print(f"{FAIL} GitHub web-interface commits must not be merged:")
            for sha, subject in bad:
                print(f"         {sha[:12]} {subject}")
            print("       These are created when you accept review suggestions "
                  "in the browser.")
            print("       Squash each one into the commit it corrects with "
                  f"'git rebase -i {self.base_branch}'.")
            print(f"       See: {DOCS_URL}")
            return False
        print(f"{PASS} No GitHub web-interface commits.")
        return True

    def check_wip_commits(self, commits: list) -> bool:
        '''reject commits whose subject marks them as unfinished, e.g.
           "WIP: ...", "[DEBUG] ...", "TMP: ..."'''
        bad = []
        for sha, subject in commits:
            match = WIP_MARKER_RE.match(subject)
            if match is None:
                continue
            marker = next(g for g in match.groups() if g is not None)
            if marker.upper() in WIP_MARKERS:
                bad.append((sha, subject))
        if bad:
            print(f"{FAIL} Commits marked as unfinished must not be merged:")
            for sha, subject in bad:
                print(f"         {sha[:12]} {subject}")
            print("       Finish the work and reword the subject, or drop the "
                  "commit from the branch.")
            print(AMEND_HINT)
            print(f"       See: {DOCS_URL}")
            return False
        print(f"{PASS} No WIP/DEBUG/TMP commits.")
        return True

    def check_commit_lengths(self, commits: list) -> bool:
        ok = True
        for sha, subject in commits:
            if len(subject) > MAX_SUBJECT_LEN:
                print(f"{FAIL} {sha[:12]} subject too long ({len(subject)} "
                      f"chars, limit {MAX_SUBJECT_LEN}): {subject}")
                print(AMEND_HINT)
                ok = False
        if ok:
            print(f"{PASS} All commit subject lines within "
                  f"{MAX_SUBJECT_LEN} characters.")
        return ok

    def check_author_emails(self) -> bool:
        emails = self.run_git(
            ["log", f"{self.base_branch}..HEAD", "--format=%ae"],
        ).strip()
        bad = []
        for email in emails.splitlines():
            if "example.com" in email:
                bad.append(email)
        if bad:
            print(f"{FAIL} Author email(s) with example.com are not allowed:")
            for email in sorted(set(bad)):
                print(f"         {email}")
            print("       Set a real address with 'git config user.email' and "
                  "rewrite the commits.")
            return False
        print(f"{PASS} No unacceptable author emails.")
        return True

    def _blob_ends_with_newline(self, blob: str) -> bool | None:
        '''return whether the blob ends with a newline byte;
           None if the blob is missing or empty'''
        result = subprocess.run(
            ["git", "cat-file", "blob", blob],
            capture_output=True,
        )
        if result.returncode != 0:
            return None
        data = result.stdout
        if not data:
            return None
        return data.endswith(b"\n")

    def check_trailing_newlines(self) -> bool:
        '''check that no changed source file loses its trailing newline;
           some editors strip the final newline by default, which shows up
           as "No newline at end of file" noise in GitHub review.
           Only files the PR adds or modifies are checked, and a modified
           file is only flagged if the base version did end with a newline,
           so pre-existing violations do not block unrelated edits.
        '''
        merge_base = self.run_git(
            ["merge-base", self.base_branch, "HEAD"],
        ).strip()
        # -z so that paths are not quoted, --raw for the modes and blob ids
        result = subprocess.run(
            ["git", "diff", "-z", "--raw", "--no-abbrev", "-M",
             f"{merge_base}..HEAD"],
            capture_output=True, check=True,
        )
        # records are ":oldmode newmode oldblob newblob status\0path\0",
        # with a second path for renames and copies
        fields = result.stdout.decode("utf-8", "surrogateescape").split("\0")

        ok = True
        i = 0
        while i < len(fields) - 1:
            old_mode, new_mode, old_blob, new_blob, status = \
                fields[i].lstrip(":").split(" ")
            npaths = 2 if status[0] in "RC" else 1
            new_path = fields[i + npaths]
            i += 1 + npaths
            if status[0] not in "AMR":
                # deletions, copies, type changes etc.
                continue
            if new_mode not in REGULAR_FILE_MODES:
                continue
            if pathlib.Path(new_path).suffix.lower() not in SOURCE_EXTENSIONS:
                continue

            new_ends = self._blob_ends_with_newline(new_blob)
            if new_ends is None or new_ends:
                # empty file, or trailing newline present
                continue

            if status[0] != "A":
                if (old_mode not in REGULAR_FILE_MODES or
                        not self._blob_ends_with_newline(old_blob)):
                    # base version already lacked a trailing newline
                    continue
                print(f"{FAIL} {new_path} loses its trailing newline in this PR.")
            else:
                print(f"{FAIL} {new_path} is added without a trailing newline.")
            ok = False

        if ok:
            print(f"{PASS} No changed source files lose their trailing newline.")
        else:
            print("       Configure your editor to end files with a newline "
                  "(e.g. VSCode \"files.insertFinalNewline\": true).")
        return ok

    def run(self) -> None:
        if not self.base_branch:
            self.base_branch = self.run_git(
                ["merge-base", "HEAD", self.DEFAULT_UPSTREAM],
            ).strip()
            print(f"Using merge base with {self.DEFAULT_UPSTREAM}: "
                  f"{self.base_branch}")

        commits = self.commit_subjects()

        print(f"\nChecking {len(commits)} commit(s) since "
              f"{self.base_branch}...\n")

        if not commits:
            print(f"{SKIP} Branch adds no commits; nothing to check.")
            sys.exit(0)

        results = [
            self.check_merge_commits(),
            self.check_fixup_commits(commits),
            self.check_github_ui_commits(commits),
            self.check_wip_commits(commits),
            self.check_commit_lengths(commits),
            self.check_author_emails(),
            self.check_trailing_newlines(),
        ]

        failures = results.count(False)
        print("\n" + ('All checks passed.' if not failures
                      else f'{failures} check(s) failed.'))
        sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Check PR branch commit conventions",
    )
    parser.add_argument(
        "--base-branch",
        default=None,
        help="Upstream base branch or commit to compare against "
             "(default: merge base of HEAD with origin/master)",
    )
    args = parser.parse_args()
    CheckBranchConventions(args.base_branch).run()
