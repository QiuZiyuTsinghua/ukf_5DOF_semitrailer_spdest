# Makefile for UKF Semitrailer State Estimator Test
# 半挂卡车UKF状态估计器测试程序编译文件

# 编译器设置 / Compiler settings
CXX = g++
CXXFLAGS = -std=c++11 -O2 -Wall -Wextra
INCLUDES = -Iinclude -I/usr/include/eigen3

# Eigen库路径 (如果需要) / Eigen library path (if needed)
# EIGEN_PATH = /usr/include/eigen3
# INCLUDES += -I$(EIGEN_PATH)

# 目标文件 / Target executable
TARGET = build/test_ukf
SIMPLE_TARGET = build/simple_test
MINIMAL_TARGET = build/minimal_test

# 源文件 / Source files
MAIN_SOURCES = tests/main.cpp src/semitrailer_dynamics.cpp src/ukf_estimator.cpp
SIMPLE_SOURCES = tests/simple_test.cpp src/semitrailer_dynamics.cpp src/ukf_estimator.cpp
MINIMAL_SOURCES = tests/minimal_test.cpp src/semitrailer_dynamics.cpp src/ukf_estimator.cpp

# 头文件依赖 / Header dependencies
HEADERS = include/semitrailer_dynamics.h include/ukf_estimator.h

# 默认目标 / Default target
all: $(TARGET)

# 编译所有测试程序 / Build all test programs
all-tests: $(TARGET) $(SIMPLE_TARGET) $(MINIMAL_TARGET)

# 编译主测试程序 / Build main test executable
$(TARGET): $(MAIN_SOURCES) $(HEADERS) | build
	@echo "编译主测试程序... / Building main test program..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(MAIN_SOURCES) -o $(TARGET)
	@echo "主测试程序编译完成！/ Main test program build completed!"

# 编译简单测试程序 / Build simple test executable
$(SIMPLE_TARGET): $(SIMPLE_SOURCES) $(HEADERS) | build
	@echo "编译简单测试程序... / Building simple test program..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SIMPLE_SOURCES) -o $(SIMPLE_TARGET)
	@echo "简单测试程序编译完成！/ Simple test program build completed!"

# 编译最小测试程序 / Build minimal test executable
$(MINIMAL_TARGET): $(MINIMAL_SOURCES) $(HEADERS) | build
	@echo "编译最小测试程序... / Building minimal test program..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(MINIMAL_SOURCES) -o $(MINIMAL_TARGET)
	@echo "最小测试程序编译完成！/ Minimal test program build completed!"

# 创建build目录 / Create build directory
build:
	@mkdir -p build

# 清理编译文件 / Clean build files
clean:
	@echo "清理编译文件... / Cleaning build files..."
	rm -rf build/*
	@echo "清理完成！/ Clean completed!"

# 运行主测试 / Run main test
run: $(TARGET)
	@echo "运行UKF估计器测试... / Running UKF estimator test..."
	./$(TARGET)

# 运行简化测试 / Run simple test
run-simple: $(SIMPLE_TARGET)
	@echo "运行简化测试... / Running simple test..."
	./$(SIMPLE_TARGET)

# 运行最小测试 / Run minimal test
run-minimal: $(MINIMAL_TARGET)
	@echo "运行最小测试... / Running minimal test..."
	./$(MINIMAL_TARGET)
simple: simple_test
	@echo "运行简化测试... / Running simple test..."
	timeout 30s ./simple_test || echo "测试被超时终止 / Test terminated by timeout"

# 编译简化测试 / Build simple test
simple_test: simple_test.o semitrailer_dynamics.o ukf_estimator.o
	@echo "编译简化测试... / Building simple test..."
	$(CXX) $(CXXFLAGS) simple_test.o semitrailer_dynamics.o ukf_estimator.o -o simple_test

# 调试模式编译 / Debug build
debug: CXXFLAGS += -g -DDEBUG
debug: $(TARGET)

# 显示帮助信息 / Show help
help:
	@echo "可用命令 / Available commands:"
	@echo "  make         - 编译程序 / Build program"
	@echo "  make run     - 编译并运行 / Build and run"
	@echo "  make debug   - 调试模式编译 / Debug build"
	@echo "  make clean   - 清理编译文件 / Clean build files"
	@echo "  make help    - 显示帮助 / Show help"

# 声明伪目标 / Declare phony targets
.PHONY: all clean run debug help
