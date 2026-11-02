#!/usr/bin/env python3

"""Build, generate, load, and package ThunderScope LiteX artifacts."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request


ROOT = Path(__file__).resolve().parent
PROJECT = "thunderscope"
LITEX_RELEASE_TAG = "2024.12"
PYTHON_VERSION = (3, 12)
BUILD_PATH = ROOT / "build"
DIST_PATH = ROOT / "distrib"
VENV_PATH = ROOT / ".venv"
VENV_PYTHON = VENV_PATH / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
LITEX_PATH = VENV_PATH / "litex-src"
STAMP = VENV_PATH / f".stamp_{LITEX_RELEASE_TAG}"
RELEASE_VARIANTS = ("dev", "prod")
BETA_VARIANTS = ("a50t", "a100t", "a200t")
ALL_VARIANTS = RELEASE_VARIANTS + BETA_VARIANTS


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "target",
        nargs="?",
        default="help",
        help="venv, dev, prod, a50t, a100t, a200t, all, release, driver, docs, "
        "gen, gen-<variant>, load-<variant>, or distclean",
    )
    parser.add_argument(
        "--vivado-settings",
        type=Path,
        help="Path to Vivado settings64.bat (Windows); otherwise search standard Xilinx locations.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print build commands without running them or modifying the environment.",
    )
    return parser.parse_args()


def ensure_python_version():
    if os.name != "nt":
        return
    version = f"{PYTHON_VERSION[0]}.{PYTHON_VERSION[1]}"
    if sys.version_info[:2] == PYTHON_VERSION:
        print(f"Using Python {sys.version.split()[0]}")
        return

    launcher = shutil.which("py")
    if launcher is None:
        raise RuntimeError(
            f"Python {version} is required on Windows. Install Python {version} and "
            "the Python Launcher, or run this script with the required version directly."
        )

    command = [launcher, f"-V:{version}", str(Path(__file__).resolve()), *sys.argv[1:]]
    print(f"Restarting under Python {version} using {launcher}.", flush=True)
    try:
        os.execv(launcher, command)
    except OSError as error:
        raise RuntimeError(f"Could not restart the build script with Python 3.12: {error}") from error


def find_vivado_settings(explicit=None):
    candidates = []
    if explicit is not None:
        candidates.append(explicit)
    if os.environ.get("VIVADO_SETTINGS64_BAT"):
        candidates.append(Path(os.environ["VIVADO_SETTINGS64_BAT"]))
    if os.environ.get("XILINX_VIVADO"):
        xilinx = Path(os.environ["XILINX_VIVADO"])
        candidates.extend([xilinx, xilinx / "settings64.bat"])

    candidates.extend(
        [
            Path(r"C:\Xilinx\Vivado\2019.2\settings64.bat"),
            Path(r"C:\Xilinx\Vivado\settings64.bat"),
            Path(os.environ.get("ProgramFiles", r"C:\Program Files"))
            / "Xilinx" / "Vivado" / "settings64.bat",
            Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
            / "Xilinx" / "Vivado" / "settings64.bat",
        ]
    )

    roots = [
        Path(r"C:\Xilinx\Vivado"),
        Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "Xilinx" / "Vivado",
        Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
        / "Xilinx" / "Vivado",
    ]
    for root in roots:
        if root.is_dir():
            candidates.extend(sorted(root.glob("*/settings64.bat")))

    for candidate in candidates:
        path = candidate / "settings64.bat" if candidate.is_dir() else candidate
        if path.is_file():
            return path.resolve()
    searched = "\n  ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        "Vivado settings64.bat was not found. Pass --vivado-settings or set "
        "VIVADO_SETTINGS64_BAT. Searched:\n  " + searched
    )


class CommandRunner:
    def __init__(self, dry_run=False, vivado_settings=None):
        self.dry_run = dry_run
        self.vivado_settings = vivado_settings
        self.env = os.environ.copy()
        self.env["PYTHONUTF8"] = "1"
        self.env["LITEX_RELEASE_TAG"] = LITEX_RELEASE_TAG

    def run(self, args, cwd=ROOT, vivado=False):
        args = [str(arg) for arg in args]
        if vivado and os.name == "nt":
            if self.vivado_settings is None:
                raise RuntimeError("Vivado settings64.bat is required for this command")
            command = (
                f'call "{self.vivado_settings}" && '
                f"{subprocess.list2cmdline(args)}"
            )
            printable = f"cmd.exe /d /c {command}"
        else:
            command = args
            printable = subprocess.list2cmdline(args) if os.name == "nt" else " ".join(
                subprocess.list2cmdline([arg]) for arg in args
            )

        print(f"> {printable}")
        if not self.dry_run:
            subprocess.run(
                command,
                cwd=cwd,
                env=self.env,
                check=True,
                shell=vivado and os.name == "nt",
            )


def git_output(*args, default=""):
    result = subprocess.run(
        ["git", *args],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else default


def build_version():
    tag_commit = git_output("rev-list", "--abbrev-commit", "--tags", "--max-count=1")
    tag = (
        git_output("describe", "--abbrev=0", "--tags", tag_commit)
        if tag_commit
        else ""
    )
    commit = git_output("rev-parse", "--short", "HEAD", default="unknown")
    date = git_output("log", "-1", "--format=%cd", "--date=format:%Y%m%d", default="unknown")

    if tag:
        version = tag[1:] if tag.startswith("v") else tag
        if commit != tag_commit:
            version = f"{version}-next-{commit}-{date}"
    else:
        version = f"{commit}-{date}"
    if git_output("status", "--porcelain"):
        version += "-dirty"
    return version


def bootstrap_python():
    if os.name == "nt":
        launcher = shutil.which("py")
        if launcher is None:
            raise RuntimeError(
                f"Python Launcher is required on Windows (py -V:{PYTHON_VERSION[0]}.{PYTHON_VERSION[1]})"
            )
        return [launcher, f"-V:{PYTHON_VERSION[0]}.{PYTHON_VERSION[1]}"]
    return [shutil.which("python3") or sys.executable]


def stamp_is_current():
    if not STAMP.is_file():
        return False
    stamp_time = STAMP.stat().st_mtime
    for path in (ROOT / "requirements.txt", ROOT / "version.mk"):
        if path.exists() and path.stat().st_mtime > stamp_time:
            return False
    return True


def ensure_venv(runner):
    if runner.dry_run:
        print(f"> ensure LiteX {LITEX_RELEASE_TAG} environment at {VENV_PATH}")
        return

    bootstrap = bootstrap_python()
    if not VENV_PYTHON.is_file():
        runner.run([*bootstrap, "-m", "venv", VENV_PATH], vivado=False)
    if stamp_is_current():
        runner.run([VENV_PYTHON, "-c", f"import {PROJECT}"], vivado=False)
        print(f"Using virtual environment, LiteX version: {LITEX_RELEASE_TAG}")
        return

    runner.run([VENV_PYTHON, "-m", "pip", "install", "--upgrade", "pip"])
    runner.run(
        [VENV_PYTHON, "-m", "pip", "install", "-r", ROOT / "requirements.txt"]
    )
    LITEX_PATH.mkdir(parents=True, exist_ok=True)
    setup_script = LITEX_PATH / "litex_setup.py"
    if not setup_script.is_file():
        print(f"Downloading LiteX setup script to {setup_script}")
        urllib.request.urlretrieve(
            "https://raw.githubusercontent.com/enjoy-digital/litex/master/litex_setup.py",
            setup_script,
        )
    setup_code = (
        "import litex_setup; "
        "litex_setup.litex_setup_import_repos(download=True); "
        f"litex_setup.litex_setup_init_repos(tag={LITEX_RELEASE_TAG!r}); "
        "litex_setup.litex_setup_install_repos()"
    )
    runner.run([VENV_PYTHON, "-c", setup_code], cwd=LITEX_PATH)
    runner.run([VENV_PYTHON, "-c", f"import {PROJECT}"])
    STAMP.touch()


def run_thunderscope(runner, *args):
    runner.run(
        [VENV_PYTHON, ROOT / f"{PROJECT}.py", *args],
        vivado=True,
    )


def build_variant(runner, variant):
    output_dir = BUILD_PATH / f"{PROJECT}_{variant}"
    run_thunderscope(
        runner,
        f"--variant={variant}",
        "--build",
        f"--output-dir={output_dir}",
    )
    if runner.dry_run:
        return
    distribution = DIST_PATH / variant
    distribution.mkdir(parents=True, exist_ok=True)
    gateware = output_dir / "gateware"
    version = build_version()
    for source_name, target_name in (
        (f"{PROJECT}_full.bin", f"{PROJECT}_full_{variant}_{version}.bin"),
        (f"{PROJECT}_full.mcs", f"{PROJECT}_full_{variant}_{version}.mcs"),
        (f"{PROJECT}_update.bin", f"{PROJECT}_update_{variant}_{version}.bin"),
        (f"{PROJECT}_update.bit", f"{PROJECT}_update_{variant}_{version}.bit"),
    ):
        source = gateware / source_name
        target = distribution / target_name
        print(f"Copying {source} -> {target}")
        shutil.copy2(source, target)


def generate_variant(runner, variant):
    run_thunderscope(
        runner,
        f"--variant={variant}",
        "--gen",
        f"--output-dir={BUILD_PATH / f'{PROJECT}_{variant}'}",
    )


def load_variant(runner, variant):
    run_thunderscope(
        runner,
        f"--variant={variant}",
        "--load",
        f"--output-dir={BUILD_PATH / f'{PROJECT}_{variant}'}",
    )


def build_driver(runner):
    run_thunderscope(
        runner,
        "--driver",
        f"--driver-dir={BUILD_PATH / PROJECT / 'driver'}",
        f"--csr-csv={BUILD_PATH / PROJECT / 'csr.csv'}",
    )


def build_docs(runner):
    run_thunderscope(runner, "--variant=prod", "--doc")
    runner.run(
        [
            VENV_PYTHON,
            "-m",
            "sphinx",
            BUILD_PATH / PROJECT / "doc",
            BUILD_PATH / "docs",
        ]
    )
    if not runner.dry_run:
        DIST_PATH.mkdir(parents=True, exist_ok=True)
        archive = shutil.make_archive(
            str(DIST_PATH / f"{PROJECT}_docs"),
            "zip",
            root_dir=ROOT,
            base_dir=BUILD_PATH / "docs",
        )
        print(f"Created {archive}")


def run_target(target, runner):
    if target == "help":
        return False
    if target == "distclean":
        if runner.dry_run:
            print(f"> remove {BUILD_PATH} and {DIST_PATH}")
        else:
            for path in (BUILD_PATH, DIST_PATH):
                if path.exists():
                    shutil.rmtree(path)
        return True

    ensure_venv(runner)
    if target == "venv":
        return True
    if target in ALL_VARIANTS:
        build_variant(runner, target)
    elif target.startswith("gen-") and target[4:] in RELEASE_VARIANTS:
        generate_variant(runner, target[4:])
    elif target == "gen":
        for variant in RELEASE_VARIANTS:
            generate_variant(runner, variant)
    elif target.startswith("load-") and target[5:] in ALL_VARIANTS:
        load_variant(runner, target[5:])
    elif target == "driver":
        build_driver(runner)
    elif target == "docs":
        build_docs(runner)
    elif target == "release":
        for variant in RELEASE_VARIANTS:
            build_variant(runner, variant)
        build_docs(runner)
        build_driver(runner)
        if not runner.dry_run:
            source = BUILD_PATH / PROJECT / "doc"
            destination = DIST_PATH / "doc"
            print(f"Copying {source} -> {destination}")
            shutil.copytree(source, destination, dirs_exist_ok=True)
    elif target == "all":
        for variant in ALL_VARIANTS:
            build_variant(runner, variant)
        build_docs(runner)
        build_driver(runner)
    else:
        return False
    return True


def main():
    ensure_python_version()
    args = parse_args()
    valid_targets = {
        "help", "venv", "all", "release", "driver", "docs", "gen", "distclean",
        *ALL_VARIANTS,
        *(f"gen-{variant}" for variant in RELEASE_VARIANTS),
        *(f"load-{variant}" for variant in ALL_VARIANTS),
    }
    if args.target not in valid_targets:
        print(f"Unknown target: {args.target}", file=sys.stderr)
        print("Run `python build.py help` to list targets.", file=sys.stderr)
        return 2
    if args.target == "help":
        print(
            "Targets: venv, dev, prod, a50t, a100t, a200t, all, release, "
            "driver, docs, gen, gen-dev, gen-prod, load-<variant>, distclean"
        )
        print("Options: --vivado-settings PATH, --dry-run")
        return 0

    vivado_settings = None
    if os.name == "nt" and args.target not in ("venv", "distclean"):
        vivado_settings = find_vivado_settings(args.vivado_settings)
        print(f"Using Vivado settings: {vivado_settings}")
    runner = CommandRunner(args.dry_run, vivado_settings)
    try:
        return 0 if run_target(args.target, runner) else 2
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"Build failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
