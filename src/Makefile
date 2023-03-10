##########################################
#  Makefile
##########################################

#
# Nom du programme
#
PROG = xpbd_prog
SPRING = spring
CLOTH = cloth
WALL = wall
BALL = ball

#
# Fichiers sources (NE PAS METTRE les .h ni les .o seulement les .cpp)
#
SOURCES = main.cpp pbd.cpp particle.cpp constraint.cpp constraint_dist.cpp sphere.cpp constraint_fixed.cpp cloth.cpp constraint_isobending.cpp constraint_bending.cpp constraint_wall.cpp constraint_penetration.cpp
SPRING_SOURCES = spring.cpp pbd.cpp particle.cpp constraint.cpp constraint_dist.cpp sphere.cpp constraint_fixed.cpp constraint_isobending.cpp constraint_bending.cpp constraint_wall.cpp constraint_penetration.cpp
CLOTH_SOURCES = demo_cloth.cpp pbd.cpp particle.cpp constraint.cpp constraint_dist.cpp sphere.cpp constraint_fixed.cpp cloth.cpp constraint_isobending.cpp constraint_bending.cpp constraint_wall.cpp constraint_penetration.cpp
WALL_SOURCES = wall.cpp pbd.cpp particle.cpp constraint.cpp constraint_dist.cpp sphere.cpp constraint_fixed.cpp cloth.cpp constraint_isobending.cpp constraint_bending.cpp constraint_wall.cpp constraint_penetration.cpp
BALL_SOURCES = ball.cpp pbd.cpp particle.cpp constraint.cpp constraint_dist.cpp sphere.cpp constraint_fixed.cpp constraint_isobending.cpp constraint_bending.cpp constraint_wall.cpp constraint_penetration.cpp


#
# Fichiers objets (ne pas modifier sauf si l'extension n'est pas .cpp)
#
BUILDDIR = ../build
OBJETS = ${SOURCES:%.cpp=$(BUILDDIR)/%.o}
SPRING_OBJETS = ${SPRING_SOURCES:%.cpp=$(BUILDDIR)/%.o}
CLOTH_OBJETS = ${CLOTH_SOURCES:%.cpp=$(BUILDDIR)/%.o}
WALL_OBJETS = ${WALL_SOURCES:%.cpp=$(BUILDDIR)/%.o}
BALL_OBJETS = ${BALL_SOURCES:%.cpp=$(BUILDDIR)/%.o}


#
# Compilateur C++
#
CXX = g++

#
# Options du compilateur C++
#   -g pour debugger, -O optimise, -Wall affiche les erreurs, -I pour les headers
#   -std=c++11 pour C++11
# Exemple: CXXFLAGS= -std=c++11 -Wall -O -I/usr/local/qt/include
#
CXXFLAGS = -std=c++11 -Wall -O -I ../
CXX_FLAGS +=
CXX_FLAGS += -m64 -march=native
CXX_FLAGS += -fopt-info-vec-optimized #-fopt-info-vec-missed -ftree-vectorize
#
# Options de l'editeur de liens
#
LDFLAGS = 

#
# Librairies a utiliser
# Exemple: LDLIBS = -L/usr/local/qt/lib -lqt
#
LDLIBS = -lGL -lGLU -lglut -lpthread 


##########################################
#
# Regles de construction/destruction des .o et de l'executable
# depend-${PROG} sera un fichier contenant les dependances
#
 
all: ${PROG} ${SPRING} ${CLOTH} ${WALL} ${BALL}

run: ${PROG}
	./${PROG}

${PROG}: depend-${PROG} ${OBJETS}
	${CXX} -o $@ ${CXXFLAGS} ${LDFLAGS} ${OBJETS} ${LDLIBS}

${SPRING}: depend-${SPRING} ${SPRING_OBJETS}
	${CXX} -o $@ ${CXXFLAGS} ${LDFLAGS} ${SPRING_OBJETS} ${LDLIBS}

${CLOTH}: depend-${CLOTH} ${CLOTH_OBJETS}
	${CXX} -o $@ ${CXXFLAGS} ${LDFLAGS} ${CLOTH_OBJETS} ${LDLIBS}

${WALL}: depend-${WALL} ${WALL_OBJETS}
	${CXX} -o $@ ${CXXFLAGS} ${LDFLAGS} ${WALL_OBJETS} ${LDLIBS}

${BALL}: depend-${BALL} ${BALL_OBJETS}
	${CXX} -o $@ ${CXXFLAGS} ${LDFLAGS} ${BALL_OBJETS} ${LDLIBS}

clean:
	-@$(RM) -r $(BUILDDIR) depend-${PROG} depend-${SPRING} depend-${CLOTH} depend-${WALL} depend-${BALL} core 1>/dev/null 2>&1

clean-all: clean
	-@$(RM) ${PROG} -${SPRING} -${CLOTH} -${WALL} -${BALL} 1>/dev/null 2>&1
  

# Gestion des dependances : creation automatique des dependances en utilisant 
# l'option -MM de g++ (attention tous les compilateurs n'ont pas cette option)
#
depend-${PROG}:
	${CXX} ${CXXFLAGS} -MM ${SOURCES} > depend-${PROG}

depend-${SPRING}:
	${CXX} ${CXXFLAGS} -MM ${SPRING_SOURCES} > depend-${SPRING}

depend-${CLOTH}:
	${CXX} ${CXXFLAGS} -MM ${CLOTH_SOURCES} > depend-${CLOTH}

depend-${WALL}:
	${CXX} ${CXXFLAGS} -MM ${WALL_SOURCES} > depend-${WALL}

depend-${BALL}:
	${CXX} ${CXXFLAGS} -MM ${BALL_SOURCES} > depend-${BALL}
###########################################
#
# Regles implicites
#

.SUFFIXES: .cpp .cxx .c

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

$(BUILDDIR)/%.o: %.cpp
	mkdir -p $(dir $@)
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<


.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.c.o:
	$(CC) -c (CFLAGS) $(INCPATH) -o $@ $<


#############################################
#
# Inclusion du fichier des dependances
#
-include depend-${PROG}
-include depend-${SPRING}
-include depend-${CLOTH}
-include depend-${WALL}
-include depend-${BALL}