VENV := .venv
PIP  := $(VENV)/bin/pip

pip-install:
	python -m venv $(VENV)
	$(PIP) install deps/pyReedsShepp
	$(PIP) install -e .

venv:
	@if [ ! -d $(VENV) ]; then $(MAKE) pip-install; fi
	@echo "source $(VENV)/bin/activate"

clean:
	rm -rf $(VENV) __pycache__ src/**/__pycache__ src/*.egg-info deps/pyReedsShepp/*.egg-info deps/pyReedsShepp/*.so deps/pyReedsShepp/build/ *.mp4

.PHONY: pip-install venv clean
