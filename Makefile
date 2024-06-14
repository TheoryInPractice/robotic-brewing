RM = /bin/rm
MV = /bin/mv
CP = /bin/cp
PYTHON = python3
MKDIR = mkdir
GCOV = gcov
LCOV = lcov
GENHTML = genhtml
TAR = gtar

# OS detection
ifeq '$(findstring ;,$(PATH))' ';'
    detected_OS := Windows
else
    detected_OS := $(shell uname 2>/dev/null || echo Unknown)
    detected_OS := $(patsubst CYGWIN%,Cygwin,$(detected_OS))
    detected_OS := $(patsubst MSYS%,MSYS,$(detected_OS))
    detected_OS := $(patsubst MINGW%,MSYS,$(detected_OS))
endif

ifeq ($(detected_OS),Darwin)  # Mac OS X
    OPEN = open
else
    OPEN = echo "Created:"
endif

# Gurobi settings
ifeq ("$(CI)","true")
    export GUROBI_ON=false
else
    export GUROBI_ON=true
    ifndef GUROBI_HOME
        # Mac Gurobi v11.0.1
        ifneq ("$(wildcard /Library/gurobi1101/macos_universal2)","")
            export GUROBI_HOME=/Library/gurobi1101/macos_universal2
        else
            $(error GUROBI_HOME is not set)
        endif
    endif
endif

# Profiler settings
export PROFILE_ON ?= false

CMAKE=cmake
CMAKE_OPTS=-DCMAKE_C_COMPILER=$(CC) -DCMAKE_CXX_COMPILER=$(CXX)

SRC_CPP=src/main/cpp
TEST_CPP=src/test/cpp
PROJ_DIR=$(PWD)
BUILD_DIR=$(PROJ_DIR)/build
DIST_DIR=$(PROJ_DIR)/dist
TEST_BIN_DIR=$(BUILD_DIR)/test
TEST_EXEC=$(TEST_BIN_DIR)/ip_test
COV_CPP_DIR=coverage/cpp
COV_CPP=$(COV_CPP_DIR)/lcov.info
COV_PY_DIR=coverage/py
COV_PY=$(COV_PY_DIR)/lcov.info
COV_HTML=coverage/html
COV_MERGED=coverage/lcov.info

SRC_PY=src/main/python
TEST_PY=src/test/python
STUB_PY=src/stubs
IP_SOLVER_BIN=ip-solver
REDUCE_POI_BIN=reduce-poi
FILTER_POI_BIN=filter-poi
MERGE_WALKS_BIN=merge-walks
COMPUTE_BOUNDS_BIN=compute-bounds

export PYTHONPATH=$(PWD)/$(SRC_PY)
export MYPYPATH=$(PWD)/$(STUB_PY)

build: $(IP_SOLVER_BIN) $(REDUCE_POI_BIN) $(FILTER_POI_BIN) $(MERGE_WALKS_BIN)

build-pre:
	cd $(SRC_CPP) && $(CMAKE) -DCMAKE_BUILD_TYPE=Release -S . -B "$(BUILD_DIR)/Release" $(CMAKE_OPTS)

build-debug:
	cd $(SRC_CPP) && $(CMAKE) -DCMAKE_BUILD_TYPE=Debug -S . -B "$(BUILD_DIR)/Debug" $(CMAKE_OPTS)
	cd $(SRC_CPP) && $(CMAKE) --build "$(BUILD_DIR)/Debug"
	@echo "Created: build/Debug/$(IP_SOLVER_BIN)"

$(IP_SOLVER_BIN): build-pre
	cd $(SRC_CPP) && $(CMAKE) --build "$(BUILD_DIR)/Release" --target $(IP_SOLVER_BIN)
	@echo "Created: build/Release/$(IP_SOLVER_BIN)"

$(REDUCE_POI_BIN): build-pre
	cd $(SRC_CPP) && $(CMAKE) --build "$(BUILD_DIR)/Release" --target $(REDUCE_POI_BIN)
	@echo "Created: build/Release/$(REDUCE_POI_BIN)"

$(FILTER_POI_BIN): build-pre
	cd $(SRC_CPP) && $(CMAKE) --build "$(BUILD_DIR)/Release" --target $(FILTER_POI_BIN)
	@echo "Created: build/Release/$(FILTER_POI_BIN)"

$(MERGE_WALKS_BIN): build-pre
	cd $(SRC_CPP) && $(CMAKE) --build "$(BUILD_DIR)/Release" --target $(MERGE_WALKS_BIN)
	@echo "Created: build/Release/$(MERGE_WALKS_BIN)"

$(COMPUTE_BOUNDS_BIN): build-pre
	cd $(SRC_CPP) && $(CMAKE) --build "$(BUILD_DIR)/Release" --target $(COMPUTE_BOUNDS_BIN)
	@echo "Created: build/Release/$(COMPUTE_BOUNDS_BIN)"

test: test-cpp

test-cpp:
# check style
	find "$(SRC_CPP)" "$(TEST_CPP)" -regex '.*[.][ch]pp' | xargs clang-format --dry-run -Werror
# run tests
	@echo "GTEST_FILTER: $(GTEST_FILTER)"
	cd $(SRC_CPP) && $(CMAKE) -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON -S . -B "$(BUILD_DIR)/Test" $(CMAKE_OPTS) -DGUROBI_ON=$(GUROBI_ON)
	cd $(SRC_CPP) && $(CMAKE) --build "$(BUILD_DIR)/Test"
	"$(TEST_EXEC)" --output-on-failure $(GTEST_OPTS)

#-------------------------------------------------------------------------------
#    C++ Code Coverage
#-------------------------------------------------------------------------------
ifeq ($(detected_OS),Darwin)  # Mac OS X
export LLVM_PROFILE_FILE=$(TEST_BIN_DIR)/cov.profraw
LLVM_PROFILE_DATA=$(TEST_BIN_DIR)/cov.profdata

test-cpp-cov: test-cpp
	$(MKDIR) -p "$(COV_CPP_DIR)"
	xcrun llvm-profdata merge -sparse "$(LLVM_PROFILE_FILE)" -o "$(LLVM_PROFILE_DATA)"
	xcrun llvm-cov export --format=lcov "$(TEST_EXEC)" -instr-profile="$(LLVM_PROFILE_DATA)" > "${COV_CPP}"
	$(LCOV) -q -r "${COV_CPP}" "*/include/*" "*.h" "*/test/*" "*/build*/*" "*/external/*" -o "${COV_CPP}"
else
test-cpp-cov: test-cpp
	$(MKDIR) -p "$(COV_CPP_DIR)"
	$(LCOV) -q --gcov-tool $(GCOV) -d "$(TEST_BIN_DIR)" -c -o "$(COV_CPP)"
	$(LCOV) -q -r "${COV_CPP}" "*/include/*" "*.h" "*/test/*" "*/build*/*" "*/external/*" -o "${COV_CPP}"
endif
#-------------------------------------------------------------------------------

test-py:
# check style
	autopep8 --recursive --diff --exit-code "$(SRC_PY)" "$(TEST_PY)"
# check types
	mypy "$(SRC_PY)"
# run tests with coverage
	$(MKDIR) -p "$(COV_PY_DIR)"
	$(PYTHON) -m pytest -x --cov="$(SRC_PY)" --cov-report=lcov:$(COV_PY) $(PYTEST_OPTS) $(TEST_PY)

coverage: test-cpp-cov test-py
	$(LCOV) --add-tracefile $(COV_PY) -a $(COV_CPP) -o $(COV_MERGED)

coverage-html: coverage
	$(GENHTML) -o $(COV_HTML) $(COV_MERGED)
	@$(OPEN) $(COV_HTML)/index.html

clean:
	@echo "Cleaning..."
	@$(RM) -rf build/* coverage/*
	@echo "Cleaning done."

lab:
	jupyter-lab

.PHONY: build build-pre build-debug test test-cpp test-cpp-cov test-py coverage coverage-html clean lab
