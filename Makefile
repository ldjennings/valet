VENV := .venv
PIP  := $(VENV)/bin/pip

install:
	python -m venv $(VENV)
	$(PIP) install -e . 
# 	$(PIP) install -e deps/pyReedsShepp --quiet

venv:
	@if [ ! -d $(VENV) ]; then $(MAKE) install; fi
	@echo "source $(VENV)/bin/activate"

clean:
	rm -rf $(VENV) __pycache__ src/**/__pycache__ src/*.egg-info deps/pyReedsShepp/*.egg-info deps/pyReedsShepp/*.so deps/pyReedsShepp/build/ *.mp4

.PHONY: install venv clean
