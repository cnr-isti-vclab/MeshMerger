.PHONY: all clean

EXE = charity
OUTPUT_NAME = $(EXE)

CXX = g++
CXXFLAGS = -O2 -s -DNDEBUG -std=c++17 -Wno-delete-incomplete -fopenmp \
  -I./3rdParty -I/usr/include/eigen3 \
  -I./IO -I./MeshLevelSet -I./Registration -I./x64 \
  -I/usr/local/include/pcl-1.14 -I/usr/include/flann -I/usr/include/boost -I/usr/include/rapidjson \
  -I/opt/tbb/include/oneapi/tbb \
  -I/opt/tbb/include \
  -I./3rdParty/spectra_install/include \
  -I./3rdParty/teaser/ \
  -I./3rdParty/tinyply/ \
  -I/usr/share/doc/curl \
  -I./3rdParty/pmc/ 


LDFLAGS = -L/usr/lib/x86_64-linux-gnu -L/opt/tbb/gnu_13.2_cxx11_64_relwithdebinfo
LDLIBS = -lopenvdb -lpcl_common -lpcl_kdtree -lpcl_features -lpcl_io -lflann_cpp \
  -lboost_system -lboost_filesystem -lboost_iostreams -ljsoncpp -lblosc -llz4 -lz \
  -lOpenEXR  -lIex -lImath -ltbb \
  -lpcl_filters -lpcl_search


# Function to recursively find all files matching a pattern ($2) in a directory ($1)
rwildcard=$(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d))

# Use the function to find all .cpp files in 3rdParty and other directories
SOURCES := $(call rwildcard,./3rdParty/*.cpp) \
           $(wildcard ./IO/*.cpp) \
           $(wildcard ./MeshLevelSet/*.cpp) \
           $(wildcard ./Registration/*.cpp) \
		   $(wildcard ./3rdParty/teaser/*.cpp) \
		   $(wildcard ./3rdParty/teaser/*.cc) \
		   $(wildcard ./3rdParty/tinyply/*.cpp) \
		   $(wildcard ./3rdParty/pmc/*.cpp) \
           $(wildcard ./registration_main.cpp)

OBJS = $(SOURCES:.cpp=.o)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(CXX_PLATFORM_FLAGS) -c -o $@ $<
%.o: %.cc 
	$(CXX) $(CXXFLAGS) $(CXX_PLATFORM_FLAGS) -c -o $@ $<
%.o: %.c
	$(CXX) $(CXXFLAGS) $(CXX_PLATFORM_FLAGS) -c -o $@ $<
all: $(EXE)

$(EXE): $(OBJS)
	$(CXX) -o $(OUTPUT_NAME) $^ $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)


clean:
	rm -f $(EXE) $(OBJS)
