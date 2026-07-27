TARGET = Manipulation
DIAG_TARGET = DiagnoseGoalNeighborhood

SRCS =  main.cpp 
SRCS += icmMath.cpp CSpace.cpp RRT.cpp CFree.cpp CFreeICS.cpp
SRCS += Labeling.cpp Link.cpp Node.cpp OneHand.cpp PointCloud.cpp Rectangle.cpp Planner.cpp FormClosure.cpp PSO.cpp
SRCS += Robot.cpp RRTTree.cpp Shape.cpp Square.cpp Wall.cpp LShape.cpp TaskSet.cpp Problem.cpp Controller.cpp pathsmooth.cpp pathshortcut.cpp TShape.cpp Triangle.cpp SpectralUtil.cpp GraphUtils.cpp clusters.cpp SpaceConfig.cpp RRTStar.cpp GoalExpansionSearch.cpp
#  InformedRRTStar.cpp
OBJS = $(SRCS:.cpp=.o)
DIAG_OBJS = diagnose_goal_neighborhood.o $(filter-out main.o,$(OBJS))

CXX = g++
#CXXFLAGS = -g -Wall
#以下実行用
CXXFLAGS = -O3 -Wall -flto=auto -march=native -mtune=native
#以下，デバック用
#CXXFLAGS = -Wall -O0 -g -fno-inline -fno-omit-frame-pointer -mtune=native -march=native -mfpmath=both
#ここまで
INCDIR = -I/usr/include -I.

#LIBDIR = -L/usr/local/lib
#LIBS = -lompl -lmlpack

all: $(TARGET)
diagnose: $(DIAG_TARGET)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCDIR) -o $@ -c $<

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBDIR) $(LIBS)
#以下以前
# 	$(CXX) -o $@ $^ $(LIBDIR) $(LIBS)

$(DIAG_TARGET): $(DIAG_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBDIR) $(LIBS)

clean:
	rm -f *.o $(TARGET) $(DIAG_TARGET)
#以下以前
# 	rm -f *.o

depend:
	makedepend $(INCDIR) $(SRCS)

# CXXFLAGS = -flto -Wall -O3 -mtune=native -march=native -mfpmath=both
