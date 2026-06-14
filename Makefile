.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := build

# =========================
# User-overridable inputs  ##
# =========================
ROS_DISTRO      ?= jazzy
BUILD_TYPE      ?= Release
TEST            ?= OFF
PACKAGES        ?=
NUM_WORKERS     ?=
CMAKE_ARGS      ?=
ROSDEP_SKIP_KEYS ?=
LOCAL_DEPS_PREFIX ?= $(CURDIR)/.deps
GTSAM_POINTS_PREFIX ?= /usr/local
LOCAL_INSTALL_RPATH ?= $(GTSAM_POINTS_PREFIX)/lib;/usr/local/cuda-11.8/lib64

BAG_ROOT        ?= /media/roar/data/rosbags/putnam/may_26
DATA_ROOT       ?= ./dlio_data
RUN             ?= run_5
RAW_BAG         ?= $(BAG_ROOT)/$(RUN)/filtered/all
ORIGIN_RUN      ?=
UTM_ORIGIN_FILE ?=
MAP_RUN         ?=
RVIZ            ?= false

VIEWS           ?= overview,low,top,start
HTTP_PORT       ?= 8000
MAP_WEB         ?= $(DATA_ROOT)/map_web
MAX_POINTS      ?= 127000000

# =========================
# Common paths / groups   ##
# =========================
ROS_SETUP       := /opt/ros/$(ROS_DISTRO)/setup.bash

PKG_CORE          := dlio
PKG_GLIM          := glim_ros
PKG_LOCALIZATION  := gicp_localization
PKG_TO_TEST       := dlio
PKG_ALL           := $(PKG_CORE) $(PKG_GLIM) $(PKG_LOCALIZATION)

PREPPED         = $(DATA_ROOT)/$(RUN)_prepped
DUMP            = $(DATA_ROOT)/$(RUN)_dump
MAP             = $(DATA_ROOT)/$(RUN)_map.pcd
LOC             = $(DATA_ROOT)/$(RUN)_loc
REF_RUN         = $(if $(strip $(MAP_RUN)),$(MAP_RUN),$(RUN))
REF_MAP         = $(DATA_ROOT)/$(REF_RUN)_map.pcd
REF_UTM         = $(DATA_ROOT)/$(REF_RUN)_dump/T_world_utm.txt

ROSDEP_SKIP_ARG = $(if $(strip $(ROSDEP_SKIP_KEYS)),--skip-keys "$(ROSDEP_SKIP_KEYS)")

# =========================
# Helper macros           ##
# =========================
define _source
	set -eo pipefail
	if [ ! -f "$(ROS_SETUP)" ]; then \
		echo "ROS setup not found: $(ROS_SETUP)" >&2; \
		exit 1; \
	fi
	source "$(ROS_SETUP)"
	if [ -f "$(GTSAM_POINTS_PREFIX)/lib/cmake/gtsam_points/gtsam_points-config.cmake" ]; then \
		export CMAKE_PREFIX_PATH="$(GTSAM_POINTS_PREFIX):$${CMAKE_PREFIX_PATH:-}"; \
		export LD_LIBRARY_PATH="$(GTSAM_POINTS_PREFIX)/lib:$${LD_LIBRARY_PATH:-}"; \
	fi
endef

define _source_overlay
	$(_source)
	if [ -f install/setup.bash ]; then source install/setup.bash; fi
endef

define _source_required
	$(_source)
	if [ ! -f install/setup.bash ]; then \
		echo "workspace overlay missing: install/setup.bash" >&2; \
		echo "Run: make build" >&2; \
		exit 1; \
	fi
	source install/setup.bash
endef

# $(call build_up_to,BuildType,Packages,ExtraColconArgs,ExtraCmakeArgs)
# PACKAGES (env/CLI) overrides $(2); NUM_WORKERS adds --parallel-workers and
# CMAKE_BUILD_PARALLEL_LEVEL when set.
define build_up_to
	$(_source)
	if [ -n "$$NUM_WORKERS" ]; then export CMAKE_BUILD_PARALLEL_LEVEL=$$NUM_WORKERS; fi
	colcon build --symlink-install $(3) $${NUM_WORKERS:+--parallel-workers $$NUM_WORKERS} \
		--cmake-args -DCMAKE_BUILD_TYPE=$(1) -DBUILD_TESTING=$(TEST) "-DCMAKE_INSTALL_RPATH=$(LOCAL_INSTALL_RPATH)" $(CMAKE_ARGS) $(4) \
		--packages-up-to $${PACKAGES:-$(2)}
endef

# $(call build_select,BuildType,Packages)
define build_select
	$(_source)
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$(1) -DBUILD_TESTING=$(TEST) "-DCMAKE_INSTALL_RPATH=$(LOCAL_INSTALL_RPATH)" $(CMAKE_ARGS) --packages-select $(2)
endef

# $(call test_common,extra_colcon_test_args)
define test_common
	$(_source_required)
	if [ -z "$$PACKAGES" ]; then \
		colcon test $(1) --packages-select $(PKG_TO_TEST); \
	else \
		colcon test $(1) --packages-up-to $$PACKAGES; \
	fi ; \
	colcon test-result --verbose
endef

# =========================
# Phony                   ##
# =========================
# Auto-derived from target names across the Makefile so the list cannot drift.
# None of these targets produce real files matching their names, so marking all
# phony is safe.
.PHONY: $(shell awk -F: '/^[a-zA-Z0-9_.%-]+:[^=]/{print $$1}' $(MAKEFILE_LIST) | sort -u)

# =========================
# Meta                    ##
# =========================
# ---- Help configuration -------------------------------------------------
HELP_WIDTH ?= 32   # column width for target names

##@ Meta
help:  ## Show help; supports PATTERN=regex, SECTION=substring, ALL=1 (include undocumented), PRIVATE=1 (include _private)
	@awk -v w="$(HELP_WIDTH)" -v pat="$(PATTERN)" -v secf="$(SECTION)" -v all="$(ALL)" -v priv="$(PRIVATE)" '\
	BEGIN{ FS=":.*##"; IGNORECASE=1; if(!w) w=28; sec=""; show=(secf==""?1:0); print ""; print "Usage: make <target> [VAR=val ...]"; print "" } \
	/^##@/ { sec=substr($$0,5); show=(secf=="" || index(sec,secf)); if(show) { print ""; print sec } ; next } \
	/^[a-zA-Z0-9_.%-]+:.*##/ { t=$$1; gsub(/^[ \t]+|[ \t]+$$/,"",t); if(!priv && t ~ /^_/) next; d=$$2; if(!show) next; if(pat!="" && index(t,pat)==0 && index(d,pat)==0) next; printf("  %-*s %s\n", w, t, d); next } \
	all=="1" && /^[a-zA-Z0-9_.%-]+:/ { t=$$1; gsub(/^[ \t]+|[ \t]+$$/,"",t); if(!priv && t ~ /^_/) next; if(!show) next; if(pat!="" && index(t,pat)==0) next; printf("  %-*s %s\n", w, t, "(undocumented)"); }' $(MAKEFILE_LIST)
	@echo ""; echo "Tip: make help-vars  (show common vars) |  make help-<pattern>  (filter by pattern)"

help-%:  ## Show help filtered by pattern (e.g., make help-pipeline)
	@$(MAKE) help PATTERN="$*"

help-vars:  ## Show common variables and current values
	@printf "\nCommon variables (override as VAR=value):\n\n"
	@printf "  %-20s  %s  (current: %s)\n" "PACKAGES"            "Package list override"             "$(PACKAGES)"
	@printf "  %-20s  %s  (current: %s)\n" "NUM_WORKERS"         "Parallel workers for build"        "$(NUM_WORKERS)"
	@printf "  %-20s  %s  (current: %s)\n" "ROS_DISTRO"          "ROS 2 distro"                      "$(ROS_DISTRO)"
	@printf "  %-20s  %s  (current: %s)\n" "BUILD_TYPE"          "CMake build type"                  "$(BUILD_TYPE)"
	@printf "  %-20s  %s  (current: %s)\n" "TEST"                "BUILD_TESTING value"               "$(TEST)"
	@printf "  %-20s  %s  (current: %s)\n" "CMAKE_ARGS"          "Extra args after --cmake-args"      "$(CMAKE_ARGS)"
	@printf "  %-20s  %s  (current: %s)\n" "GTSAM_POINTS_PREFIX" "CUDA gtsam_points install prefix"  "$(GTSAM_POINTS_PREFIX)"
	@printf "  %-20s  %s  (current: %s)\n" "LOCAL_INSTALL_RPATH" "RPATH for CUDA deps"               "$(LOCAL_INSTALL_RPATH)"
	@printf "  %-20s  %s  (current: %s)\n" "BAG_ROOT"            "Root containing run_3/run_5 bags"   "$(BAG_ROOT)"
	@printf "  %-20s  %s  (current: %s)\n" "DATA_ROOT"           "Pipeline artifact directory"        "$(DATA_ROOT)"
	@printf "  %-20s  %s  (current: %s)\n" "RUN"                 "Current logical run"                "$(RUN)"
	@printf "  %-20s  %s  (current: %s)\n" "RAW_BAG"             "Raw bag path for prep/pipeline"     "$(RAW_BAG)"
	@printf "  %-20s  %s  (current: %s)\n" "ORIGIN_RUN"          "Run whose UTM origin is reused"     "$(ORIGIN_RUN)"
	@printf "  %-20s  %s  (current: %s)\n" "MAP_RUN"             "Run providing map/UTM transform"    "$(MAP_RUN)"
	@printf "  %-20s  %s  (current: %s)\n" "RVIZ"                "Open RViz during replay"            "$(RVIZ)"
	@printf "\nDLIO++ pipeline vars:  make help-pipeline\n\n"

list:  ## List target names only
	@awk -F: '/^[a-zA-Z0-9_.%-]+:/{print $$1}' $(MAKEFILE_LIST) | sed 's/:$$//' | sort -u

check-env:  ## Print local ROS/workspace/package status
	$(_source_overlay)
	echo "ROS_DISTRO=$${ROS_DISTRO:-}"
	echo "ROS setup: $(ROS_SETUP)"
	command -v ros2
	command -v colcon
	echo
	echo "colcon packages:"
	colcon list --names-only
	echo
	if [ -f install/setup.bash ]; then \
		echo "workspace overlay: install/setup.bash"; \
		for pkg in glim glim_ext glim_ros gicp_localization dlio; do \
			ros2 pkg prefix "$$pkg" 2>/dev/null || true; \
		done; \
	else \
		echo "workspace overlay missing: install/setup.bash"; \
	fi
	echo
	if [ -f "$(GTSAM_POINTS_PREFIX)/include/gtsam_points/config.hpp" ]; then \
		echo "CUDA gtsam_points:"; \
		grep -E "GTSAM_POINTS_VERSION_STRING|GTSAM_POINTS_USE_CUDA|GTSAM_POINTS_CUDA_VERSION" "$(GTSAM_POINTS_PREFIX)/include/gtsam_points/config.hpp" || true; \
	else \
		echo "CUDA gtsam_points: not found at $(GTSAM_POINTS_PREFIX)"; \
	fi
	echo
	if find /usr /usr/local \( -name IridescenceConfig.cmake -o -name iridescence-config.cmake \) -print -quit 2>/dev/null | grep -q .; then \
		echo "Iridescence: found"; \
	else \
		echo "Iridescence: not found (try: make install-deps)"; \
	fi

check-data:  ## Check current RAW_BAG and reference map inputs
	@if [ -e "$(RAW_BAG)" ]; then echo "RAW_BAG ok: $(RAW_BAG)"; else echo "RAW_BAG missing: $(RAW_BAG)" >&2; exit 1; fi
	@if [ -n "$(strip $(MAP_RUN))" ]; then \
		[ -e "$(REF_MAP)" ] || { echo "reference map missing: $(REF_MAP)" >&2; exit 1; }; \
		[ -e "$(REF_UTM)" ] || { echo "reference UTM transform missing: $(REF_UTM)" >&2; exit 1; }; \
		echo "reference map ok: $(REF_MAP)"; \
		echo "reference UTM ok: $(REF_UTM)"; \
	fi

# =========================
# Dependencies            ##
# =========================
##@ Dependencies
install-deps: install-glim-apt  ## Install apt, GLIM, rosdep, and Python deps for local Jazzy use
	sudo apt-get update
	sudo apt-get install -y python3-pip python3-colcon-common-extensions python3-rosdep libpcap-dev
	sudo rosdep init 2>/dev/null || true
	rosdep update
	$(call _source)
	rosdep install --from-paths . --ignore-src -r -y --rosdistro $(ROS_DISTRO) $(ROSDEP_SKIP_ARG)
	python3 -m pip install --user --break-system-packages mcap mcap-ros2-support pyproj numpy matplotlib

install-glim-apt:  ## Install required GLIM apt deps from Koide PPAs (Iridescence/GTSAM)
	sudo apt-get update
	sudo apt-get install -y software-properties-common
	sudo add-apt-repository -y ppa:koide3/iridescence
	sudo add-apt-repository -y ppa:koide3/gtsam
	sudo apt-get update
	sudo apt-get install -y libiridescence-dev libgtsam-no-tbb-dev

install-gtsam-points-cuda:  ## Build/install CUDA gtsam_points into GTSAM_POINTS_PREFIX
	rm -rf "$(LOCAL_DEPS_PREFIX)/src/gtsam_points" "$(LOCAL_DEPS_PREFIX)/build/gtsam_points_cuda"
	mkdir -p "$(LOCAL_DEPS_PREFIX)/src" "$(LOCAL_DEPS_PREFIX)/build"
	git clone --branch v1.2.1 --depth 1 https://github.com/koide3/gtsam_points.git "$(LOCAL_DEPS_PREFIX)/src/gtsam_points"
	# CUDA 11.8's Thrust does not provide par_nosync; par.on(stream) keeps the build compatible.
	python3 -c 'from pathlib import Path; root=Path("$(LOCAL_DEPS_PREFIX)/src/gtsam_points"); [p.write_text(t.replace("thrust::cuda::par_nosync", "thrust::cuda::par")) for p in root.rglob("*.cu") for t in [p.read_text()] if "thrust::cuda::par_nosync" in t]'
	cmake -S "$(LOCAL_DEPS_PREFIX)/src/gtsam_points" -B "$(LOCAL_DEPS_PREFIX)/build/gtsam_points_cuda" \
		-DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON -DBUILD_WITH_MARCH_NATIVE=OFF
	cmake --build "$(LOCAL_DEPS_PREFIX)/build/gtsam_points_cuda" --parallel $$(nproc)
	if [ -w "$(GTSAM_POINTS_PREFIX)" ]; then \
		cmake --install "$(LOCAL_DEPS_PREFIX)/build/gtsam_points_cuda" --prefix "$(GTSAM_POINTS_PREFIX)"; \
	else \
		sudo cmake --install "$(LOCAL_DEPS_PREFIX)/build/gtsam_points_cuda" --prefix "$(GTSAM_POINTS_PREFIX)"; \
		sudo ldconfig; \
	fi

# =========================
# Build / Test            ##
# =========================
##@ Build
build:  ## Build core in Release  (PACKAGES overrides)
	$(call build_up_to,$(BUILD_TYPE),$(PKG_CORE),,)

build-debug: BUILD_TYPE=Debug
build-debug:  ## Build core in Debug  (PACKAGES overrides)
	$(call build_up_to,$(BUILD_TYPE),$(PKG_CORE),,)

build-all:  ## Build all packages in Release
	$(call build_up_to,$(BUILD_TYPE),$(PKG_ALL),,)

build-all-debug: BUILD_TYPE=Debug
build-all-debug:  ## Build all packages in Debug
	$(call build_up_to,$(BUILD_TYPE),$(PKG_ALL),,)

build-select:  ## Release build for explicit selection  (PACKAGES required)
	$(call build_select,$(BUILD_TYPE),$(PACKAGES))

build-select-debug: BUILD_TYPE=Debug
build-select-debug:  ## Debug build for explicit selection  (PACKAGES required)
	$(call build_select,$(BUILD_TYPE),$(PACKAGES))

build-glim:  ## Build GLIM ROS stack with dependencies
	$(call build_up_to,$(BUILD_TYPE),$(PKG_GLIM),,)

build-localization:  ## Build GICP localization with dependencies
	$(call build_up_to,$(BUILD_TYPE),$(PKG_LOCALIZATION),,)

test:  ## Run tests (default set or PACKAGES)
	$(call test_common,--return-code-on-test-failure)

test-result:  ## Show verbose colcon test results
	$(call _source)
	colcon test-result --verbose

##@ Clean
clean:  ## Remove colcon build artifacts
	rm -rf build/ install/ log/

clean-test:  ## Delete colcon test results
	$(call _source)
	colcon test-result --delete-yes

# =========================
# Pipeline                ##
# =========================
##@ Pipeline
prep:  ## Prep RAW_BAG into DATA_ROOT/RUN_prepped
	$(call _source_required)
	mkdir -p "$(DATA_ROOT)"
	args=(--input "$(RAW_BAG)" --output "$(PREPPED)")
	if [ -n "$(strip $(ORIGIN_RUN))" ]; then \
		origin_file="$(DATA_ROOT)/$(ORIGIN_RUN)_prepped/utm_origin.txt"; \
		[ -f "$$origin_file" ] || { echo "UTM origin file not found: $$origin_file" >&2; exit 1; }; \
		args+=(--utm-origin "$$(tail -n 1 "$$origin_file")"); \
	fi
	if [ -n "$(strip $(UTM_ORIGIN_FILE))" ]; then \
		[ -f "$(UTM_ORIGIN_FILE)" ] || { echo "UTM origin file not found: $(UTM_ORIGIN_FILE)" >&2; exit 1; }; \
		args+=(--utm-origin "$$(tail -n 1 "$(UTM_ORIGIN_FILE)")"); \
	fi
	python3 -u scripts/prep_bag.py "$${args[@]}"

build-map:  ## Build GLIM map from DATA_ROOT/RUN_prepped
	$(call _source_required)
	ros2 run glim_ros glim_rosbag "$(PREPPED)" \
		--ros-args -p dump_path:="$(DUMP)" -p auto_quit:=true

eval-map:  ## Evaluate GLIM dump trajectory against RTK GNSS
	$(call _source_required)
	python3 scripts/eval_traj_vs_gnss.py --dump "$(DUMP)" --bag "$(PREPPED)"

export-map:  ## Export GLIM dump to PCD
	$(call _source_required)
	ros2 run glim_ros glim_dump_to_pcd "$(DUMP)" "$(MAP)"

localize:  ## Replay RUN against REF_RUN map (MAP_RUN defaults to RUN)
	$(call _source_required)
	scripts/run_localization_replay.sh "$(PREPPED)" "$(REF_MAP)" "$(REF_UTM)" "$(LOC)" "$(RVIZ)"

pipeline:  ## Run prep/map/eval/export/localize via scripts/run_dlio_pipeline.sh
	$(call _source_required)
	args=(--raw "$(RAW_BAG)" --data-root "$(DATA_ROOT)" --run "$(RUN)" --rviz "$(RVIZ)")
	if [ -n "$(strip $(ORIGIN_RUN))" ]; then args+=(--origin-run "$(ORIGIN_RUN)"); fi
	if [ -n "$(strip $(UTM_ORIGIN_FILE))" ]; then args+=(--utm-origin-file "$(UTM_ORIGIN_FILE)"); fi
	if [ -n "$(strip $(MAP_RUN))" ]; then args+=(--map-run "$(MAP_RUN)"); fi
	scripts/run_dlio_pipeline.sh "$${args[@]}"

run5-pipeline:  ## Build/localize run_5 using BAG_ROOT and DATA_ROOT
	$(MAKE) pipeline RUN=run_5 RVIZ="$(RVIZ)" BAG_ROOT="$(BAG_ROOT)" DATA_ROOT="$(DATA_ROOT)"

validate-run3:  ## Prep run_3 with run_5 origin, localize against run_5 map
	$(MAKE) pipeline RUN=run_3 ORIGIN_RUN=run_5 MAP_RUN=run_5 RVIZ=true BAG_ROOT="$(BAG_ROOT)" DATA_ROOT="$(DATA_ROOT)"

# =========================
# Viewing / Rendering     ##
# =========================
##@ Viewing
view-map:  ## Open GLIM offline viewer for DATA_ROOT/RUN_dump
	$(call _source_required)
	ros2 run glim_ros offline_viewer "$(DUMP)"

render-map:  ## Render static map views to DATA_ROOT/renders
	$(call _source_required)
	python3 scripts/render_map.py --map "$(MAP)" --out "$(DATA_ROOT)/renders" \
		--traj "$(DUMP)/traj_imu.txt" --views "$(VIEWS)"

viz-cache:  ## Show cached LOC/live_error.csv in RViz without rerunning GICP
	$(call _source_required)
	scripts/show_cached_error_viz.sh "$(LOC)" "$(REF_MAP)"

export-map-html:  ## Generate browser viewer folder for MAP
	$(call _source_required)
	python3 scripts/export_map_html.py \
		--map "$(MAP)" \
		--out "$(MAP_WEB)" \
		--traj "$(DUMP)/traj_imu.txt" \
		--max-points "$(MAX_POINTS)"

serve-map:  ## Serve MAP_WEB at http://localhost:HTTP_PORT
	python3 -m http.server "$(HTTP_PORT)" -d "$(MAP_WEB)"

stop-map-server:  ## Stop python http.server for HTTP_PORT
	pkill -f "http.server $(HTTP_PORT)" || true
