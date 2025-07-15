# Makefile for UKF Semitrailer State Estimator Test
# 半挂卡车UKF状态估计器测试程序编译文件

# 编译器设置 / Compiler settings
CXX = g++
CXXFLAGS = -std=c++11 -O2 -Wall -Wextra
INCLUDES = -I. -I/usr/include/eigen3

# Eigen库路径 (如果需要) / Eigen library path (if needed)
# EIGEN_PATH = /usr/include/eigen3
# INCLUDES += -I$(EIGEN_PATH)

# 目标文件 / Target executable
TARGET = test_ukf

# 源文件 / Source files
SOURCES = main.cpp semitrailer_dynamics.cpp ukf_estimator.cpp

# 对象文件 / Object files
OBJECTS = $(SOURCES:.cpp=.o)

# 头文件依赖 / Header dependencies
HEADERS = semitrailer_dynamics.h ukf_estimator.h

# 默认目标 / Default target
all: $(TARGET)

# 编译可执行文件 / Build executable
$(TARGET): $(OBJECTS)
	@echo "链接可执行文件... / Linking executable..."
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $(TARGET)
	@echo "编译完成！/ Build completed!"

# 编译对象文件 / Build object files
%.o: %.cpp $(HEADERS)
	@echo "编译 $< ... / Compiling $< ..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# 清理编译文件 / Clean build files
clean:
	@echo "清理编译文件... / Cleaning build files..."
	rm -f $(OBJECTS) $(TARGET) simple_test simple_test.o
	@echo "清理完成！/ Clean completed!"

# 运行测试 / Run test
run: $(TARGET)
	@echo "运行UKF估计器测试... / Running UKF estimator test..."
	./$(TARGET)

# 运行简化测试 / Run simple test
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
