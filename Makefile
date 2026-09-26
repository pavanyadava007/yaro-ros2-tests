IMAGE ?= yaro-ros2-tests:jazzy
RUN = docker run --rm --network none -v $(CURDIR)/results:/ws/results -v $(CURDIR)/scripts:/ws/scripts:ro $(IMAGE) bash -c
SETUP = . /ws/ros2_ws/install/setup.sh && cd /ws/ros2_ws
MODELS = /ws/models/yaro_0808 /ws/models/yaro_1105 /ws/models/yaro_1115 /ws/models/yaro_1310 /ws/models/yaro_1608

.PHONY: image test audit bench report all space
image:
	docker build -f docker/Dockerfile -t $(IMAGE) .

# colcon build runs in the image; this runs every gtest and the launch_testing suite and records the result.
test: image
	$(RUN) '$(SETUP) && colcon test --event-handlers console_direct+ ; colcon test-result --verbose ; python3 /ws/scripts/collect_tests.py /ws/results/tests.json'

audit: image
	$(RUN) '$(SETUP) && ros2 run yaro_check yaro_audit --datasheet /ws/models/datasheet.txt $(MODELS) > /ws/results/audit.json'

# Single thread, nothing else running on the machine. Records the CPU with the numbers.
bench: image
	$(RUN) '$(SETUP) && ros2 run yaro_check yaro_bench /ws/models/yaro_1105 > /ws/results/bench_yaro_1105.json && lscpu | sed -n "s/^Model name: *//p" > /ws/results/bench_cpu.txt'

report:
	python3 scripts/make_report.py

all: test audit bench report

# Static Hugging Face Space in site/; the JavaScript FK is checked against the C++ library first.
space:
	python3 scripts/build_space.py
	node scripts/check_space_fk.mjs
