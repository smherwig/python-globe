PKG= globe

build:
	python setup.py build

# install under $HOME/.local
install: build
	pip install --user .

uninstall:
	pip uninstall $(PKG)

clean:
	rm -rf build

.PHONY: build install clean
