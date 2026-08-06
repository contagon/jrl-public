pybuild:
    touch pyproject.toml
    uv --verbose sync --all-extras
    cp -r .venv/lib/python*/site-packages/jrl/_core.pyi python/jrl/