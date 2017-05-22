CXX=g++
CXXFLAGS=-std=c++11 -W -Wall -O3
LDFLAGS=-framework OpenGL -lglew -lglfw -lopencv_core -lopencv_highgui -lopencv_calib3d 
EXEC=application
INCDIR=include
SRCDIR=sources
OBJDIR=objects
SRC=$(wildcard $(SRCDIR)/*.cpp)
OBJ=$(SRC:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)


all: $(EXEC)

$(EXEC): $(OBJ)
	@$(CXX) -o $@ $^ $(LDFLAGS)
	@echo "Linking complete!"

$(OBJDIR):
	@mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	@$(CXX) -I $(INCDIR) -o $@ -c $< $(CXXFLAGS)
	@echo "Compiled "$<" successfully!"

.PHONY: clean mrproper

clean:
	@rm -rf $(OBJDIR)/*.o
	@echo "Cleanup complete!"

mrproper: clean
	@rm -rf $(EXEC)
	@echo "Executable removed!"
