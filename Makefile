run:
	export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python && python main.py
run_debug:
	kernprof -l -v main.py
test:
	pytest -s

