# Valet — RBE 550 Motion Planning

Hybrid A\* motion planning simulator for four vehicle types: holonomic point robot, differential drive, Ackermann car, and car-with-trailer. Built with Python, Pygame, and Shapely.

## Requirements

- Python 3.12+
- A C++ compiler (GCC or Clang) and the Boost headers — needed to build the bundled `reeds_shepp` extension

Install Boost headers:

```sh
# Debian/Ubuntu
sudo apt install libboost-dev

# macOS
brew install boost

# Fedora/RHEL
sudo dnf install boost-devel
```

The Reeds-Shepp library is vendored under `deps/pyReedsShepp` and will be built automatically during installation.

## Installation

### With uv (recommended)

[uv](https://docs.astral.sh/uv/) handles virtual environment creation and dependency resolution automatically:

```sh
uv sync
```

The `runsim` entry point is registered by `pyproject.toml` and runs the simulator directly:

```sh
uv run runsim [bot_type] [options]
```

### Without uv

Create and activate a virtual environment, then install:

```sh
python3.12 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate

pip install -e deps/pyReedsShepp  # build and install the Reeds-Shepp extension
pip install -e .                  # install the project and remaining dependencies
```

With the virtual environment active, `runsim` is available on `PATH` the same way:

```sh
runsim [bot_type] [options]
```

## Usage

```
runsim [bot_type] [options]
```

`bot_type` is one of `point`, `diff`, `car`, `trailer` (default: `diff`).

| Flag | Effect |
|---|---|
| `-m` / `--manual` | Drive the bot manually with the keyboard instead of running the planner. |
| `-r` / `--record` | Record the simulation to `recording.mp4`. |
| `-s SEED` / `--seed SEED` | Fix the RNG seed for a reproducible obstacle layout. |

### Examples

```sh
# run the trailer bot with a fixed seed
uv run runsim trailer -s 12345

# record the car bot
uv run runsim car -r

# manually drive the differential drive bot
uv run runsim diff -m
```

## Project Structure

```
src/          Python source (planner, bots, environment, simulator)
deps/         Vendored dependencies (pyReedsShepp)
report/       Typst report and supporting figures
```
